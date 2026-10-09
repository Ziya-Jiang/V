#!/usr/bin/env python3
"""CLI bridge that forwards requests to the persistent robot_agent server.

Usage::

    python3 robot_agent/bridge.py <method> <base64-json-args>

The bridge never creates a RobotContext and therefore can never own hardware.
It retains the historical base64-JSON CLI while forwarding every protocol
method, including legacy ``manifest``/``plan``/``call``.
"""

from __future__ import annotations

import base64
import json
import os
import sys
from typing import Any, Dict


def _decode_args() -> Dict[str, Any]:
    if len(sys.argv) > 2:
        try:
            return json.loads(base64.b64decode(sys.argv[2]).decode("utf-8"))
        except Exception as error:  # noqa: BLE001
            return {"error": f"bad base64/json args: {error}"}
    return {}


def _fail(message: str, error: str = "") -> None:
    print(json.dumps({"ok": False, "message": message, "error": error}, ensure_ascii=False))
    sys.exit(1)


def main() -> int:
    method = sys.argv[1] if len(sys.argv) > 1 else "manifest"
    args = _decode_args()
    if "error" in args:
        _fail(args["error"], args["error"])

    try:
        from .client import DEFAULT_HOST, DEFAULT_PORT, _request

        request = {"method": method, "args": args}
        token = os.environ.get("ROBOT_AGENT_TOKEN")
        if token:
            request["token"] = token
        raw_response = _request(
            request,
            DEFAULT_HOST,
            DEFAULT_PORT,
            timeout=900,
        )
        response = json.loads(raw_response)
    except Exception as error:  # noqa: BLE001
        _fail(f"bridge error: {error}", str(error))
    print(json.dumps(response, ensure_ascii=False))
    return 0 if response.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
