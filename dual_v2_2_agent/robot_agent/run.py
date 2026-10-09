#!/usr/bin/env python3
"""Conversational entrypoint for the Agent + Skill + Tool grasping system.

Examples::

    # Plan only (no hardware): print the skill composition for a request.
    python -m robot_agent.run --instruction "帮我整理桌面" --plan-only

    # Interactive client loop (persistent server owns and executes hardware).
    python -m robot_agent.run --interactive
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys

from .context import RobotContext
from .llm import make_llm_from_env
from .planner import Agent
from .registry import build_default_registry


def build_agent(output_dir, instruction, plan_only):
    ctx = RobotContext(
        output_dir=output_dir,
        instruction=instruction,
        plan_only=plan_only,
        logger=_setup_logger(),
    )
    registry = build_default_registry(ctx)
    llm = make_llm_from_env() if not plan_only else None
    return Agent(registry, llm=llm), registry, ctx


def _setup_logger():
    logger = logging.getLogger("robot_agent")
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("%(levelname)s: %(message)s"))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
    return logger


def handle(instruction, agent, registry, ctx, plan_only):
    if plan_only:
        plan = agent.plan(instruction)
        plan_payload = plan.to_dict()
    else:
        from .client import DEFAULT_HOST, DEFAULT_PORT, _request

        plan_request = {
            "method": "plan",
            "args": {
                "instruction": instruction,
                "output_dir": str(ctx.output_dir),
            },
        }
        token = os.environ.get("ROBOT_AGENT_TOKEN")
        if token:
            plan_request["token"] = token
        plan_response = json.loads(
            _request(plan_request, DEFAULT_HOST, DEFAULT_PORT, timeout=900)
        )
        if not plan_response.get("ok"):
            raise RuntimeError(plan_response.get("error") or plan_response.get("message"))
        plan_payload = plan_response["data"]["plan"]

    print("\n" + "=" * 72)
    print(f"Request : {instruction}")
    print("Plan    : (skill composition + constraints)")
    print("=" * 72)
    print(json.dumps(plan_payload, ensure_ascii=False, indent=2))

    if plan_only:
        print("\n[plan-only] hardware not touched. Run without --plan-only to execute.")
        return plan

    execute_request = {
        "method": "execute",
        "args": {
            "planId": plan_response["data"]["planId"],
            "confirmation": "ENABLE_DUAL_V2_2_HARDWARE",
        },
    }
    if token:
        execute_request["token"] = token
    execute_response = json.loads(
        _request(execute_request, DEFAULT_HOST, DEFAULT_PORT, timeout=900)
    )
    if not execute_response.get("ok"):
        raise RuntimeError(
            execute_response.get("error") or execute_response.get("message")
        )
    print(f"\nTask queued: {execute_response['data']['taskId']}")
    return plan_payload


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Agent + Skill + Tool grasping demo")
    parser.add_argument("--instruction", default="帮我整理桌面", help="natural-language request")
    parser.add_argument("--output", default=None, help="output directory (default: RobotRAM/output/scene)")
    parser.add_argument("--plan-only", action="store_true", help="plan without bootstrapping hardware")
    parser.add_argument("--interactive", action="store_true", help="enter a chat loop")
    args = parser.parse_args(argv)

    agent, registry, ctx = build_agent(args.output, args.instruction, args.plan_only)

    if args.interactive:
        print("RobotAgent chat loop. Type a request (Ctrl+C / 'quit' to exit).")
        while True:
            try:
                instruction = input("\n> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not instruction or instruction.lower() in {"quit", "exit", "q"}:
                break
            try:
                handle(instruction, agent, registry, ctx, args.plan_only)
            except Exception as error:  # noqa: BLE001
                print(f"error: {error}")
        return 0

    try:
        handle(args.instruction, agent, registry, ctx, args.plan_only)
    except Exception as error:  # noqa: BLE001
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
