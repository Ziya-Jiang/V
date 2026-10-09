#!/usr/bin/env python3
"""Thin client for the persistent robot_agent server.

Does NOT load any model -- it just forwards one JSON request to the running
server and prints the JSON response. This is what the DSH plugin shells out to
on every tool call, so the per-call cost is a few tens of milliseconds instead
of minutes of model loading.

Usage::

    python -m robot_agent.client <method> <base64-json-args>
"""

from __future__ import annotations

import base64
import json
import os
import socket
import sys
import time

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = int(os.environ.get("ROBOTRAM_BRIDGE_PORT", "15555"))
CONNECT_RETRIES = 30
CONNECT_RETRY_DELAY_S = 1.0


def _request(request: dict, host: str, port: int, timeout: float) -> str:
    last_error: Exception | None = None
    for _ in range(CONNECT_RETRIES):
        try:
            sock = socket.create_connection((host, port), timeout=timeout)
            try:
                sock.sendall((json.dumps(request) + "\n").encode("utf-8"))
                sock.shutdown(socket.SHUT_WR)
                buf = b""
                while True:
                    chunk = sock.recv(65536)
                    if not chunk:
                        break
                    buf += chunk
                return buf.decode("utf-8").strip()
            finally:
                sock.close()
        except (ConnectionRefusedError, ConnectionResetError, OSError) as error:
            last_error = error
            time.sleep(CONNECT_RETRY_DELAY_S)
    raise ConnectionError(f"could not reach robot_agent server at {host}:{port}: {last_error}")


def main() -> int:
    method = sys.argv[1] if len(sys.argv) > 1 else "manifest"
    raw = sys.argv[2] if len(sys.argv) > 2 else "e30="
    try:
        args = json.loads(base64.b64decode(raw).decode("utf-8"))
    except Exception as error:  # noqa: BLE001
        print(json.dumps({"ok": False, "message": f"bad base64/json args: {error}", "error": str(error)}))
        return 1

    try:
        response = _request({"method": method, "args": args}, DEFAULT_HOST, DEFAULT_PORT, timeout=900)
    except Exception as error:  # noqa: BLE001
        print(json.dumps({"ok": False, "message": str(error), "error": str(error)}))
        return 1

    print(response)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
