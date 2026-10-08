# Launching, resource paths and the Windows executable

## One way to start the GUI

| you have | start it with |
|---|---|
| a source checkout | `python -m rocket_sim` (or double-click `run_simulator.bat`) |
| the packaged build | `RocketSimulator.exe` |

`rocketsim gui` is the same code (`rocket_sim.ui.launcher.launch_gui`); there is no second launcher.
`python -m rocket_sim <command> ...` with a command (`simulate`, `motors`, ...) runs the command-line tool; with no arguments or with
`--version` / `--check` / `--self-test` it is the launcher (`rocket_sim/__main__.py`).

The launcher checks the data files and PySide6 first and turns the failures a user can fix into a message (dialog, stderr and
`<workspace>/logs/gui_error.log`), not a traceback. `run_simulator.bat` finds `.venv` next to itself, tells you the exact commands if
it does not exist or does not have the simulator installed, shows errors (no hidden console) and keeps the window open on failure.

## Resource paths (`rocket_sim/resources.py`)

Two locations, never mixed up:

| | what | source checkout / editable | PyInstaller build |
|---|---|---|---|
| **resource root** (read-only) | `data/motors`, `configs`, `vehicles` | the repository root | `<dist>/RocketSimulator/_internal/resources` |
| **workspace** (written) | `output/` (GUI runs, benchmarks), `experiments/`, `logs/` | the repository root | `Documents\RocketSimulator` |

Resolution order for the resource root: `ROCKETSIM_HOME` (must contain `data/motors`, else an error naming it) -> the bundle (frozen)
-> the checkout containing the package -> a checkout above the working directory (a regular install run from inside a clone) ->
a `ResourceError` that says what to do. `ROCKETSIM_WORKSPACE` overrides the workspace. No module computes repository paths
itself (a test enforces it); `config.loader.PROJECT_ROOT` is an alias of `resources.project_root()` and relative `motor.file` /
`vehicle_file` paths in a config are still resolved against the config's folder first, then the resource root.

**Not supported:** a plain (non-editable) `pip install .` outside a clone: the wheel does not carry `data/`, `configs/` or `vehicles/`,
so there is no resource root (the launcher says so and names `ROCKETSIM_HOME`). Use `pip install -e .` or the executable.

## Building the Windows executable

```powershell
pip install -e ".[build]"                 # once: adds PyInstaller
python scripts\build_windows.py --test    # build, then run the GUI self-test from a clean copy of the result
python scripts\build_windows.py --zip     # (optional) dist\RocketSimulator-<version>-win64.zip
```

Result: `dist\RocketSimulator\RocketSimulator.exe` (a folder distribution, about 330 MB / 1200 files; start-up a few seconds).

**Folder, not one file**: a one-file executable would unpack the same ~330 MB to a temp folder on every start (slow; antivirus
tools stall on it) and hides the bundled data from inspection. Zip the folder to share it. Nothing prevents a one-file build later.

What is packaged: Python runtime, dependencies (NumPy, SciPy, PyArrow, Matplotlib, PySide6, PyYAML), the `rocket_sim` package and
`data/motors/*.eng`, `configs/*`, `vehicles/*` (as `resources/`). What is **not**: tests, docs, `validation_data`, `validation_results`,
outputs, pandas/netCDF4/RocketPy (developer-only). The executable's file properties carry the version from `rocket_sim.version`.
The `.exe` is windowed (no console); errors appear in a dialog and in `Documents\RocketSimulator\logs\gui_error.log`.

### Testing it (what `--test` does)

The build copies the finished folder to a temp directory (no repository, no virtual environment, `PYTHONPATH` removed), runs
`RocketSimulator.exe --self-test report.json` there, and fails the build unless the report says the GUI started, listed the vehicles
and motors, ran the default vehicle and saved its results. `--self-test` drives the same `MainWindow` the user sees, off-screen.
Result of the last check: identical apogee (918.47 m) from source and from the executable.

## Limitations

* **Windows only**, unsigned (SmartScreen warns; code signing is not set up). Not built or tested on macOS/Linux.
* Built and tested with Python 3.11 only; other versions are untested.
* The executable is the GUI: it has no console, so command-line subcommands (`batch`, `validate-registry`, ...) are for the source install.
  The validation protocol also fingerprints the physics *source*, which a bundle does not contain.
* No installer, no auto-update, no custom icon.
* Bundled vehicles and configs are read-only defaults; to use your own, use *Browse* in the vehicle box (any path) rather than editing the bundle.
* A real display was used only for manual launch checks (window opened, title and responsiveness verified); the automated
  tests run the GUI off-screen.
