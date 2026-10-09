# Derivation: barometric altitude from static pressure

The flight log gives the static pressure p [Pa]. The importer needs the height above the pad, so it converts every pressure to an
altitude relative to the pad pressure p₀ (the mean pressure before launch). The conversion is the standard-atmosphere
barometric formula; this note derives it.

## 1. Hydrostatic equilibrium

A column of still air balances its weight with the pressure change:

    dp = −ρ g dh

Ideal gas: ρ = p / (R T), with R = 287.05 J/(kg K) for dry air, so

    dp / p = − (g / (R T)) dh

## 2. Temperature profile

In the troposphere (the first 11 km) the standard atmosphere has a constant lapse rate L = 0.0065 K/m:

    T(h) = T₀ − L h,    T₀ = 288.15 K (the temperature at the reference level)

Substitute T(h) and integrate from the reference level (h = 0, p = p₀) to height h (pressure p):

    ∫ dp/p = − (g/R) ∫ dh / (T₀ − L h)

The left side is ln(p/p₀). The right side integrates to (g / (R L)) ln((T₀ − L h)/T₀). Therefore

    p / p₀ = (1 − L h / T₀)^(g / (R L))

Solve for h:

    h = (T₀ / L) [ 1 − (p / p₀)^(R L / g) ]                         ← **your TODO** (`pressure_to_altitude_m`)

The exponent R L / g = 287.05287 · 0.0065 / 9.80665 = 0.190263.

Hand-calculated check used by the test: p₀ = 101325 Pa, p = 100000 Pa:

    p/p₀ = 0.986923,  ln = −0.013163,  × 0.190263 = −0.0025044,  exp = 0.997499,  1 − that = 0.0025006
    h = (288.15 / 0.0065) × 0.0025006 = 110.88 m

(The repository's own `ISAAtmosphere.pressure_to_altitude` gives 110.886 m for the same pressure because it uses geopotential height;
the test tolerance is 0.05 m.) Also p = p₀ gives h = 0, and h grows as p falls.

## 3. What this ignores, and why it matters

* **Reference pressure and temperature.** The formula gives height above the level whose pressure is p₀ and whose temperature is
  T₀. Using the pre-launch mean pressure as p₀ makes the result height above the pad, but a pad that is warmer or colder than
  288.15 K than the standard atmosphere makes every height scale wrong by (T_actual / T₀): 10 K warmer is about 3.5 % taller for
  the same pressure drop. At 400 m that is about 14 m. The importer reports the pad temperature if the log has it but does not
  correct for it unless asked.
* **Pressure noise and bias.** 1 Pa is about 0.085 m near sea level; a 30 Pa sensor offset is irrelevant once you subtract the pad
  value, but pressure noise in the log is not.
* **Dynamic pressure and bay pressure.** The sensor sees the pressure inside the avionics bay, not the free-stream static
  pressure: venting lag and high-speed port effects show up in boost (the existing flight comparisons note the same effect in
  other logs). That is a measurement error this formula cannot remove.
