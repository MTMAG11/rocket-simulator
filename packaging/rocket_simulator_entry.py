"""PyInstaller entry point of RocketSimulator.exe (the same code path as ``python -m rocket_sim``)."""

import sys

from rocket_sim.__main__ import main

if __name__ == "__main__":
    sys.exit(main())
