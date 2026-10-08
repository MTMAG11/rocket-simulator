"""Build the Windows distribution: ``dist/RocketSimulator/RocketSimulator.exe`` (PyInstaller, folder layout).

One command, from the repository root, in the project's virtual environment::

    pip install -e ".[build]"          # once: adds PyInstaller
    python scripts/build_windows.py     # build
    python scripts/build_windows.py --test   # build, copy to a clean temp folder, run the GUI self-test from there
    python scripts/build_windows.py --zip    # also write dist/RocketSimulator-<version>-win64.zip

Why a folder and not one file: a one-file executable unpacks ~200 MB of Qt/NumPy/SciPy into a temp folder on every start
(slow, and the first thing antivirus tools stall on). The folder starts faster, is easier to debug, and its bundled data
(``_internal/resources``) can be inspected. Zip it to hand it to someone.

Only what the program needs is packaged: the Python runtime and dependencies, the ``rocket_sim`` package, and the default
resources (``data/motors``, ``configs``, ``vehicles``), never the whole repository (no tests, docs, validation data, outputs).
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rocket_sim.version import SIM_VERSION  # noqa: E402

NAME = "RocketSimulator"
ENTRY = ROOT / "packaging" / "rocket_simulator_entry.py"
DIST = ROOT / "dist"
BUILD = ROOT / "build" / "pyinstaller"
RESOURCES = {  # source folder -> folder inside the bundle's resources/
    "data/motors": "resources/data/motors",
    "configs": "resources/configs",
    "vehicles": "resources/vehicles",
}
EXCLUDES = [  # present in a developer environment, never needed by the GUI
    "pytest",
    "IPython",
    "jupyter",
    "pandas",
    "netCDF4",
    "rocketpy",
    "tkinter",
    "mypy",
    "ruff",
    "psutil",
]


def _fail(msg: str) -> None:
    print(f"\nbuild failed: {msg}", file=sys.stderr)
    raise SystemExit(1)


def _version_file() -> Path:
    """Windows 'Details' tab of the .exe (version comes from rocket_sim.version, not typed here)."""
    parts = [int(x) for x in SIM_VERSION.split(".")[:3]] + [0] * (4 - len(SIM_VERSION.split(".")[:3]))
    tup = tuple(parts[:4])
    text = f"""VSVersionInfo(
  ffi=FixedFileInfo(filevers={tup}, prodvers={tup}, mask=0x3f, flags=0x0, OS=0x40004, fileType=0x1, subtype=0x0, date=(0, 0)),
  kids=[
    StringFileInfo([StringTable('040904B0', [
      StringStruct('CompanyName', ''),
      StringStruct('FileDescription', 'Autonomous Rocket Simulator'),
      StringStruct('FileVersion', '{SIM_VERSION}'),
      StringStruct('InternalName', '{NAME}'),
      StringStruct('OriginalFilename', '{NAME}.exe'),
      StringStruct('ProductName', 'Autonomous Rocket Simulator'),
      StringStruct('ProductVersion', '{SIM_VERSION}')])]),
    VarFileInfo([VarStruct('Translation', [1033, 1200])])
  ]
)
"""
    BUILD.mkdir(parents=True, exist_ok=True)
    p = BUILD / "version_info.txt"
    p.write_text(text, encoding="utf-8")
    return p


def build() -> Path:
    if sys.platform != "win32":
        _fail("this build script targets Windows (PyInstaller does not cross-compile)")
    try:
        import PyInstaller.__main__ as pyi
    except ImportError:
        _fail(
            'PyInstaller is not installed in this environment.\nInstall it with:  pip install -e ".[build]"'
        )
    for src in RESOURCES:
        if not (ROOT / src).is_dir():
            _fail(f"missing resource folder {ROOT / src}; run this script from a complete checkout")
    if not ENTRY.exists():
        _fail(f"missing entry script {ENTRY}")
    for d in (DIST / NAME, BUILD):
        shutil.rmtree(d, ignore_errors=True)

    args = [
        str(ENTRY),
        "--name",
        NAME,
        "--onedir",
        "--windowed",  # no console window; errors are shown in a dialog and written to <workspace>/logs
        "--noconfirm",
        "--clean",
        "--distpath",
        str(DIST),
        "--workpath",
        str(BUILD),
        "--specpath",
        str(BUILD),
        "--paths",
        str(ROOT / "src"),
        "--collect-submodules",
        "rocket_sim",
        "--hidden-import",
        "mpl_toolkits.mplot3d",  # the 3-D tab selects it by name (projection='3d')
        "--version-file",
        str(_version_file()),
    ]
    for src, dst in RESOURCES.items():
        args += ["--add-data", f"{ROOT / src}{';'}{dst}"]
    for m in EXCLUDES:
        args += ["--exclude-module", m]
    print("running PyInstaller:", " ".join(args[:12]), "...")
    pyi.run(args)
    exe = DIST / NAME / f"{NAME}.exe"
    if not exe.exists():
        _fail(f"PyInstaller finished but {exe} does not exist")
    print(f"\nbuilt: {exe}")
    return exe


def smoke_test(exe: Path) -> None:
    """Copy the distribution to a clean folder (no source tree, no venv on PATH) and run the GUI self-test there."""
    with tempfile.TemporaryDirectory(prefix="rocketsim_clean_") as tmp:
        clean = Path(tmp) / NAME
        shutil.copytree(exe.parent, clean)
        report = Path(tmp) / "report.json"
        ws = Path(tmp) / "workspace"
        env = {
            k: v
            for k, v in __import__("os").environ.items()
            if k.upper() not in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV", "ROCKETSIM_HOME")
        }
        env["ROCKETSIM_WORKSPACE"] = str(ws)  # keep the test's results out of the user's Documents folder
        cmd = [str(clean / f"{NAME}.exe"), "--self-test", str(report)]
        print("clean-directory test:", " ".join(cmd))
        r = subprocess.run(cmd, cwd=tmp, env=env, timeout=300)
        if not report.exists():
            _fail(f"the executable wrote no self-test report (exit code {r.returncode})")
        rep = json.loads(report.read_text(encoding="utf-8"))
        print(
            json.dumps(
                {
                    k: rep.get(k)
                    for k in ("ok", "version", "launched_as", "frozen", "apogee_m", "result_files")
                },
                indent=2,
            )
        )
        if r.returncode != 0 or not rep.get("ok"):
            _fail(f"self-test failed: {rep.get('error', 'see report')}\n{rep.get('traceback', '')}")
        print("clean-directory test passed")


def make_zip() -> Path:
    z = DIST / f"{NAME}-{SIM_VERSION}-win64.zip"
    with zipfile.ZipFile(z, "w", zipfile.ZIP_DEFLATED) as zf:
        for f in sorted((DIST / NAME).rglob("*")):
            if f.is_file():
                zf.write(f, f.relative_to(DIST))
    print(f"zip: {z} ({z.stat().st_size / 1e6:.0f} MB)")
    return z


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument(
        "--test", action="store_true", help="after building, run the GUI self-test from a clean copy"
    )
    p.add_argument("--zip", action="store_true", help="also write a zip of the distribution folder")
    a = p.parse_args()
    exe = build()
    if a.test:
        smoke_test(exe)
    if a.zip:
        make_zip()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
