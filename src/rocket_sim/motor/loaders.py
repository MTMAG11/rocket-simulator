""".eng (RASP) and CSV motor loaders, with a registry so other formats can be added.

RASP .eng format (ThrustCurve.org): ';' comment lines, then a header line
    name  diameter[mm]  length[mm]  delays  propellant[kg]  total[kg]  manufacturer
followed by 'time[s] thrust[N]' rows. A file may hold several motors (a new header line starts
the next one); use ``designation`` to pick one.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

from ..errors import MotorError
from .motor import Motor

Loader = Callable[..., Motor]
_REGISTRY: dict[str, Loader] = {}


def register_loader(extension: str, loader: Loader) -> None:
    """Register ``loader(path, designation=None, **meta) -> Motor`` for a file extension."""
    _REGISTRY[extension.lower().lstrip(".")] = loader


def _is_number(token: str) -> bool:
    try:
        float(token)
    except ValueError:
        return False
    return True


def parse_eng(text: str, source: str = "") -> list[Motor]:
    motors: list[Motor] = []
    header: list[str] | None = None
    rows: list[tuple[float, float]] = []

    def flush() -> None:
        if header is None:
            return
        motors.append(_build_eng_motor(header, rows, source))

    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith(";"):
            continue
        tok = line.split()
        if len(tok) == 2 and _is_number(tok[0]) and _is_number(tok[1]):
            if header is None:
                raise MotorError(f"{source}:{lineno}: thrust data before motor header")
            rows.append((float(tok[0]), float(tok[1])))
        elif len(tok) >= 6 and _is_number(tok[1]):
            flush()
            header, rows = tok, []
        else:
            raise MotorError(f"{source}:{lineno}: unrecognised line in .eng file: {line!r}")
    flush()
    if not motors:
        raise MotorError(f"{source}: no motor found in file")
    return motors


def _build_eng_motor(header: list[str], rows: list[tuple[float, float]], source: str) -> Motor:
    name = header[0]
    try:
        diameter = float(header[1]) / 1000.0
        length = float(header[2]) / 1000.0
        # header[3] is the delay string, which may be e.g. "4-7-10" or "P" (plugged)
        prop = float(header[4])
        total = float(header[5])
    except (ValueError, IndexError) as exc:
        raise MotorError(f"{source}: malformed header for motor {name!r}: {header}") from exc
    maker = " ".join(header[6:])
    if not rows:
        raise MotorError(f"{source}: motor {name!r} has no thrust data")
    times = [r[0] for r in rows]
    thrust = [r[1] for r in rows]
    if times[0] > 0.0:
        times.insert(0, 0.0)
        thrust.insert(0, 0.0)
    elif times[0] < 0.0:
        raise MotorError(f"{source}: motor {name!r} has negative time stamps")
    return Motor(name, diameter, length, prop, total, times, thrust, maker, header[3], source=source)


def load_eng(path: str | Path, designation: str | None = None, **_: object) -> Motor:
    p = Path(path)
    try:
        text = p.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        raise MotorError(f"cannot read motor file {p}: {exc}") from exc
    motors = parse_eng(text, source=p.name)
    if designation is None:
        return motors[0]
    for m in motors:
        if m.designation.lower() == designation.lower():
            return m
    raise MotorError(f"{p.name}: motor {designation!r} not found (have {[m.designation for m in motors]})")


def load_csv(
    path: str | Path,
    designation: str | None = None,
    *,
    propellant_mass: float | None = None,
    total_mass: float | None = None,
    diameter: float | None = None,
    length: float | None = None,
    **_: object,
) -> Motor:
    """CSV with header 'time,thrust' (s, N). Masses/size must be supplied by the caller."""
    import csv

    p = Path(path)
    if None in (propellant_mass, total_mass, diameter, length):
        raise MotorError("CSV motors need propellant_mass, total_mass, diameter, length (SI)")
    try:
        with p.open(newline="") as f:
            rdr = csv.DictReader(f)
            if rdr.fieldnames is None or not {"time", "thrust"} <= set(rdr.fieldnames):
                raise MotorError(f"{p.name}: CSV header must contain 'time' and 'thrust'")
            rows = [(float(r["time"]), float(r["thrust"])) for r in rdr]
    except (OSError, ValueError) as exc:
        raise MotorError(f"cannot parse motor CSV {p}: {exc}") from exc
    times = [r[0] for r in rows]
    thrust = [r[1] for r in rows]
    if times and times[0] > 0.0:
        times.insert(0, 0.0)
        thrust.insert(0, 0.0)
    assert propellant_mass is not None and total_mass is not None
    assert diameter is not None and length is not None
    return Motor(
        designation or p.stem,
        diameter,
        length,
        propellant_mass,
        total_mass,
        times,
        thrust,
        source=p.name,
    )


register_loader("eng", load_eng)
register_loader("csv", load_csv)


def load_motor(path: str | Path, designation: str | None = None, **meta: object) -> Motor:
    """Load a motor by file extension via the loader registry."""
    p = Path(path)
    if not p.exists():
        raise MotorError(f"motor file not found: {p}")
    ext = p.suffix.lower().lstrip(".")
    if ext not in _REGISTRY:
        raise MotorError(f"no motor loader for '.{ext}' files (known: {sorted(_REGISTRY)})")
    return _REGISTRY[ext](p, designation, **meta)


def available_motors(directory: str | Path) -> list[Path]:
    return sorted(Path(directory).glob("*.eng"))
