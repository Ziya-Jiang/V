#!/usr/bin/env python3
"""Conversational entrypoint for the Agent + Skill + Tool grasping system.

Examples::

    # Plan only (no hardware): print the skill composition for a request.
    python -m robot_agent.run --instruction "帮我整理桌面" --plan-only

    # Interactive chat loop (bootstrap + execute when hardware is attached).
    python -m robot_agent.run --interactive
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from .context import RobotContext
from .executor import Executor
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
    plan = agent.plan(instruction)

    print("\n" + "=" * 72)
    print(f"Request : {instruction}")
    print("Plan    : (skill composition + constraints)")
    print("=" * 72)
    print(plan.to_json())

    if plan_only:
        print("\n[plan-only] hardware not touched. Run without --plan-only to execute.")
        return plan

    ctx.bootstrap()
    results = Executor(registry).run(plan, ctx)
    succeeded = sum(1 for r in results if r.ok)
    print(f"\nExecuted {succeeded}/{len(results)} skill steps.")
    return plan


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
