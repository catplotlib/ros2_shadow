from __future__ import annotations

import argparse
import sys
from pathlib import Path

from ros2_shadow import __version__
from ros2_shadow.config import ConfigError, ShadowConfig
from ros2_shadow.node import run as run_compare


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="shadow",
        description="Compare a candidate node's output against production, live.",
    )
    parser.add_argument("config", type=Path, help="shadow YAML config")
    parser.add_argument(
        "--bridge",
        action="store_true",
        help="run the domain bridge instead of the comparison; this is the process "
             "that isolates the candidate in its own ROS_DOMAIN_ID",
    )
    parser.add_argument("--version", action="version", version=f"ros2_shadow {__version__}")
    args = parser.parse_args(argv)

    if not args.config.exists():
        print(f"shadow: no such config: {args.config}", file=sys.stderr)
        return 2
    try:
        config = ShadowConfig.from_yaml(args.config)
    except (ConfigError, OSError) as exc:
        print(f"shadow: bad config: {exc}", file=sys.stderr)
        return 2

    if args.bridge:
        from ros2_shadow.bridge import run as run_bridge

        return run_bridge(config)
    return run_compare(config)


if __name__ == "__main__":
    raise SystemExit(main())
