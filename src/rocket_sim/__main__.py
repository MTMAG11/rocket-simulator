"""``python -m rocket_sim`` opens the GUI; ``python -m rocket_sim <command> ...`` runs the command-line tool (same as ``rocketsim``)."""

from __future__ import annotations

import sys

_LAUNCHER_FLAGS = {"--version", "--check", "--self-test", "-h", "--help"}


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if not args or args[0] in _LAUNCHER_FLAGS:
        from .ui.launcher import main as launcher_main

        return launcher_main(args)
    from .cli import main as cli_main

    return cli_main(args)


if __name__ == "__main__":
    raise SystemExit(main())
