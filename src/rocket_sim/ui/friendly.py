"""Plain-language text for the window: the story of a flight, friendly versions of warnings and errors. No Qt here."""

from __future__ import annotations

import math

from ..simulation.record import FlightRecord

M_TO_FT = 3.280839895
MS_TO_KMH = 3.6
MS_TO_MPH = 2.2369362921
M_TO_MILES = 1.0 / 1609.344

# Heights people can picture (metres); the story compares the rocket's highest point with the tallest one it clears.
LANDMARKS = (
    (12.0, "a school bus"),
    (30.0, "a 10-storey building"),
    (93.0, "the Statue of Liberty"),
    (330.0, "the Eiffel Tower"),
    (828.0, "the Burj Khalifa, the tallest building in the world"),
    (3776.0, "Mount Fuji"),
    (8849.0, "Mount Everest"),
)


def feet(m: float) -> str:
    return f"{m * M_TO_FT:,.0f} ft"


def height_text(m: float) -> str:
    return f"{m:,.0f} m ({feet(m)})"


def speed_text(ms: float) -> str:
    return f"{ms:,.0f} m/s ({ms * MS_TO_KMH:,.0f} km/h, {ms * MS_TO_MPH:,.0f} mph)"


def duration_text(s: float) -> str:
    if s < 90:
        return f"{s:.0f} seconds"
    return f"{int(s // 60)} min {int(round(s % 60))} s"


def compare_height(m: float) -> str:
    """'about 3 times as high as the Eiffel Tower' for the tallest landmark below ``m``; '' if nothing is smaller."""
    best = None
    for h, name in LANDMARKS:
        if h <= m:
            best = (h, name)
    if best is None:
        return ""
    ratio = m / best[0]
    if ratio < 1.25:
        return f"That is about as high as {best[1]}."
    return f"That is about {ratio:.1f} times as high as {best[1]}."


def _event(rec: FlightRecord, name: str) -> float | None:
    for e in rec.events:
        if e.name == name:
            return float(e.t)
    return None


def flight_story(rec: FlightRecord) -> list[str]:
    """What happened, as short sentences a child can read."""
    s = rec.summary
    out: list[str] = []
    ap = s.get("apogee_m")
    if not isinstance(ap, int | float) or ap < 1.0 or _event(rec, "liftoff") is None:
        return ["The rocket did not take off. Its engine was too weak for its weight, or something stopped the flight."]
    out.append(f"Your rocket climbed to {height_text(float(ap))}.")
    comp = compare_height(float(ap))
    if comp:
        out.append(comp)
    vmax = s.get("max_velocity_ms")
    if isinstance(vmax, int | float):
        mach = s.get("max_mach")
        extra = ""
        if isinstance(mach, int | float) and mach >= 0.3:
            extra = f" That is {mach * 100:.0f} % of the speed of sound."
        out.append(f"Its top speed was {speed_text(float(vmax))}.{extra}")
    t_burn = _event(rec, "burnout")
    t_rail = _event(rec, "rail_exit")
    if t_burn is not None:
        lift = _event(rec, "liftoff") or 0.0
        out.append(f"The engine pushed for {t_burn - lift:.1f} seconds, then the rocket coasted upward on its speed.")
    rail_v = s.get("rail_exit_velocity_ms")
    if t_rail is not None and isinstance(rail_v, int | float):
        out.append(f"It left the launch rail going {float(rail_v):.0f} m/s.")
    t_ap = _event(rec, "apogee")
    if t_ap is not None:
        out.append(f"It reached its highest point {duration_text(t_ap)} after the engine was lit.")
    if _event(rec, "parachute_deploy") is not None:
        out.append("At the top, the parachute opened and the rocket floated down.")
    imp = s.get("impact_speed_ms")
    dist = s.get("landing_distance_m")
    t_end = s.get("flight_time_s")
    if isinstance(imp, int | float) and isinstance(dist, int | float):
        land = f"It landed {dist:,.0f} m ({dist * M_TO_MILES:.2f} miles) from the launch pad"
        if isinstance(t_end, int | float):
            land += f", {duration_text(float(t_end))} after launch"
        out.append(land + ".")
        if imp <= 6.0:
            out.append(f"The landing was gentle ({imp:.1f} m/s), a good landing for a rocket.")
        elif imp <= 10.0:
            out.append(f"The landing was firm ({imp:.1f} m/s). A bigger parachute would make it softer.")
        else:
            out.append(f"The landing was hard ({imp:.1f} m/s). The rocket would probably be damaged; try a bigger parachute.")
    else:
        out.append("The simulation ended before the rocket landed.")
    return out


WARNING_TRANSLATIONS = (
    ("statically UNSTABLE", "This rocket is unbalanced: it can wobble and tumble because its fins are too small or its weight is in the wrong place."),
    ("never left the pad", "The rocket never left the launch pad: its engine is too weak for its weight."),
    ("reached t_max", "The flight lasted longer than the simulator was set to wait, so the landing was not shown."),
    ("stated Mach range", "The rocket went very fast. At those speeds the computer is less sure of its numbers."),
    ("angle of attack reached", "At some moment the rocket turned sideways to the air, which means it tumbled or wobbled a lot."),
    ("STRUCTURAL", "The air pushed on the rocket so hard that it might have bent or broken."),
)


def friendly_warnings(warnings: list[str]) -> list[str]:
    """Plain versions of the warnings a child can act on; technical notes are left for the Numbers tab."""
    out: list[str] = []
    for w in warnings:
        if "static margin at launch" in w and "UNSTABLE" not in w:
            out.append("This rocket is only just balanced. Bigger fins or some weight in the nose would make it steadier.")
            continue
        for key, text in WARNING_TRANSLATIONS:
            if key in w and text not in out:
                out.append(text)
    return out


def friendly_error(message: str) -> str:
    """A simulator error as one or two sentences, keeping the technical text after a blank line."""
    m = message.lower()
    plain = ""
    if "does not fit" in m and ("motor" in m or "nozzle" in m):
        plain = "This engine does not fit in this rocket. Pick a thinner engine or a wider rocket."
    elif "motor length" in m and "exceeds" in m:
        plain = "This engine is too long for this rocket. Pick a shorter engine or a taller rocket."
    elif "no such file" in m or "not found" in m:
        plain = "A file the simulator needs could not be found. If you chose an engine you downloaded, download it again."
    elif "thrust" in m and "weight" in m:
        plain = "The engine is too weak to lift this rocket."
    if not plain:
        return message
    return f"{plain}\n\nDetails for grown-ups: {message}"


def percent_of_sound(mach: float) -> str:
    return "n/a" if not math.isfinite(mach) else f"{mach * 100:.0f} % of the speed of sound"


COMPASS = ("north", "northeast", "east", "southeast", "south", "southwest", "west", "northwest")

PHASE_WORDS = {
    "PRELAUNCH": "Waiting on the pad",
    "IGNITION": "Engine lit",
    "POWERED_ASCENT": "Engine pushing",
    "BURNOUT": "Engine just stopped",
    "COAST": "Coasting upward",
    "APOGEE": "Top of the flight",
    "DESCENT": "Floating down",
    "LANDED": "Landed",
}


def compass(degrees: float) -> str:
    """The nearest of eight compass words for a bearing in degrees (0 = north, 90 = east)."""
    return COMPASS[int(round((degrees % 360.0) / 45.0)) % 8]


def phase_word(name: str) -> str:
    return PHASE_WORDS.get(name.upper(), name.replace("_", " ").capitalize())


def steering_story(rec: FlightRecord) -> list[str]:
    """A sentence about the steering engine, or nothing when its nozzle never moved."""
    try:
        y, z = rec.col("tvc_y"), rec.col("tvc_z")
    except KeyError:
        return []
    tilt = math.degrees(float(max(math.hypot(a, b) for a, b in zip(y, z, strict=True)))) if len(y) else 0.0
    if tilt < 0.05:
        return []
    return [f"The steering engine tilted its nozzle by up to {tilt:.1f} degrees to keep the rocket pointing the right way."]
