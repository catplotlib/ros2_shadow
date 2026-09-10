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

    return run_compare(config)


if __name__ == "__main__":
    raise SystemExit(main())
