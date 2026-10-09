# Derivation: centre of gravity of a set of components

The vehicle file lists components with a mass `m_i` and an axial position `x_i` (the position of that component's own centre of
gravity, measured aft of the nose tip). The simulator reports the vehicle's CG from them; the Block 0 test recomputes it
independently from your CAD mass report, so that a typing mistake in the file shows up as a disagreement.

## 1. Definition

The centre of gravity is the point about which the weight of the parts has no net moment. With gravity uniform over the rocket,
the moment of each part about the nose tip is `m_i g x_i`, and the CG position x_cg is where one weight M g (M = Σ m_i) produces
the same total moment:

    M g x_cg = Σ m_i g x_i       (g cancels)

    x_cg = Σ m_i x_i / Σ m_i                       ← **your TODO** (`center_of_gravity`)

Hand-calculated check used by the test: masses (0.2, 0.3, 0.5) kg at (0.1, 0.5, 0.9) m:

    Σ m_i x_i = 0.02 + 0.15 + 0.45 = 0.62 kg m,   Σ m_i = 1.0 kg   →   x_cg = 0.62 m

Two sanity checks: the result is never outside [min x_i, max x_i], and moving a component aft moves the CG aft in proportion
to its share of the total mass.

## 2. What it leaves out

* A component's own CG is a point only if you give it as one: a body tube or a nose cone has its CG inside it, not at its end
  (the vehicle file computes shell CGs from the geometry).
* The motor and propellant are not components of the file; the simulator adds them (the propellant burns away, so the CG moves
  forward during the burn). The check function therefore also reports the CG with the motor included.
* This is the axial position only. Lateral offsets (batteries off-axis) change the CG in y and z in the same way and make the
  inertia tensor non-diagonal.
