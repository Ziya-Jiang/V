#!/usr/bin/env python3
"""Persistent model server for the DSH bridge.

Loads the heavy models (VGGT-1B / GroundingDINO / SAM2 / RAM) and opens the
cameras + robot arm ONCE, then stays alive serving requests. The DSH plugin
talks to this server through a thin client, so each tool call is fast instead
of re-loading several GB of weights every time.

Protocol (newline-delimited JSON over TCP on 127.0.0.1:ROBOTRAM_BRIDGE_PORT):
    request  -> {"method": "manifest"|"plan"|"call", "args": {...}}
    response -> {"ok": bool, "message": str, "error": str?, "data": ...}
One request per connection. Bootstrap runs eagerly in a background thread, so
``manifest``/``plan``/``dry_run`` are served immediately while the first real
execution waits for the models to finish loading.
"""

from __future__ import annotations

import json
import os
import socket
import sys
import threading
from typing import Any, Dict

from .base import PlanError
from .context import RobotContext, write_status
from .executor import resolve_object_ref
from .registry import build_default_registry

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = int(os.environ.get("ROBOTRAM_BRIDGE_PORT", "15555"))


def _sanitize(value: Any) -> Any:
    """Recursively convert numpy scalars/arrays to JSON-safe Python values."""
    import numpy as np

    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return _sanitize(value.tolist())
    if isinstance(value, dict):
        return {str(k): _sanitize(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitize(v) for v in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


class RobotAgentServer:
    def __init__(self, output_dir=None, instruction="clean the table") -> None:
        self.ctx = RobotContext(
            output_dir=output_dir,
            instruction=instruction,
            plan_only=False,
        )
        self.registry = build_default_registry(self.ctx)
        self.ctx.load_categories_from_output()
        self.agent = None

        # Eager bootstrap in the background: by the time the user confirms a
        # real execution, the models are usually already loaded.
        self._ready = threading.Event()
        self._boot_error: Exception | None = None
        threading.Thread(target=self._bootstrap, daemon=True).start()

    def _bootstrap(self) -> None:
        try:
            self.ctx.bootstrap()
        except Exception as error:  # noqa: BLE001
            self._boot_error = error
            write_status("error", error=str(error))
        finally:
            self._ready.set()

    def _ensure_bootstrapped(self) -> None:
        self._ready.wait()
        if self._boot_error is not None:
            raise self._boot_error

    # ------------------------------------------------------------------ #
    def handle(self, request: Dict[str, Any]) -> Dict[str, Any]:
        method = request.get("method")
        args = request.get("args") or {}
        try:
            if method == "manifest":
                return self._manifest()
            if method == "plan":
                return self._plan(args)
            if method == "call":
                return self._call(args)
            return {"ok": False, "message": f"unknown method: {method}", "error": f"unknown method: {method}"}
        except PlanError as error:
            return {"ok": False, "message": str(error), "error": str(error)}
        except Exception as error:  # noqa: BLE001
            return {"ok": False, "message": f"{method} failed: {error}", "error": str(error)}

    def _manifest(self) -> Dict[str, Any]:
        return {
            "ok": True,
            "message": "manifest",
            "data": {
                "tools": self.registry.tool_manifest(),
                "skills": self.registry.skill_manifest(),
            },
        }

    def _plan(self, args: Dict[str, Any]) -> Dict[str, Any]:
        from .planner import Agent

        instruction = args.get("instruction", "clean the table")
        if self.agent is None:
            self.agent = Agent(self.registry)
        plan = self.agent.plan(instruction)
        return {"ok": True, "message": "plan", "data": {"plan": plan.to_dict()}}

    def _call(self, args: Dict[str, Any]) -> Dict[str, Any]:
        kind = args.get("kind", "tool")
        name = args.get("name")
        if not name:
            raise PlanError("missing 'name'")
        call_args = args.get("args", {}) or {}
        dry_run = bool(args.get("dry_run", True))

        # The plugin passes output_dir at the top level of the request; apply it
        # to the shared context so perception reads the scene's vlm files.
        output_dir = args.get("output_dir")
        if output_dir:
            from pathlib import Path

            self.ctx.output_dir = Path(output_dir)
            self.ctx.load_categories_from_output()

        resolved = {k: resolve_object_ref(v, self.ctx) for k, v in call_args.items()}
        target = self.registry.tool(name) if kind == "tool" else self.registry.skill(name)

        if dry_run:
            return {
                "ok": True,
                "message": f"[dry-run] {kind} {name} -- hardware not touched",
                "data": {
                    "dry_run": True,
                    "kind": kind,
                    "name": name,
                    "args": _sanitize(resolved),
                },
            }

        self._ensure_bootstrapped()
        result = target(self.ctx, **resolved)
        envelope: Dict[str, Any] = {
            "ok": result.ok,
            "message": result.message,
            "data": {
                "dry_run": False,
                "kind": kind,
                "name": name,
                "payload": _sanitize(result.payload),
            },
        }
        if result.error:
            envelope["error"] = result.error
        return envelope

    # ------------------------------------------------------------------ #
    def serve_forever(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(16)
        print(json.dumps({"ok": True, "message": "listening", "port": port}), flush=True)
        while True:
            conn, _ = srv.accept()
            threading.Thread(target=self._serve_one, args=(conn,), daemon=True).start()

    def _serve_one(self, conn: socket.socket) -> None:
        try:
            buf = b""
            while b"\n" not in buf:
                chunk = conn.recv(65536)
                if not chunk:
                    break
                buf += chunk
            request = json.loads(buf.decode("utf-8"))
            response = self.handle(request)
            conn.sendall((json.dumps(response, ensure_ascii=False) + "\n").encode("utf-8"))
        except Exception as error:  # noqa: BLE001
            try:
                conn.sendall(
                    (json.dumps({"ok": False, "message": str(error), "error": str(error)}) + "\n").encode("utf-8")
                )
            except Exception:  # noqa: BLE001
                pass
        finally:
            conn.close()


def main() -> int:
    port = DEFAULT_PORT
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            pass
    server = RobotAgentServer()
    server.serve_forever(port=port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
