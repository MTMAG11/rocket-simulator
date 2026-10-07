# Vehicle file format (`vehicle.json`, format version 1)

A rocket is described as a list of physically meaningful **components**. The simulator derives total mass, CG, the full inertia
tensor, CP, static margin, aerodynamic inputs and motor mass depletion from them. **Nothing derived is typed in.** Files are
human-readable JSON, versioned (`format_version`), deterministic (same file -> same compiled vehicle and the same SHA-256, which
is recorded in every record built from it) and strictly validated (unknown keys are errors).

Machine-readable schema: [`vehicles/vehicle.schema.json`](../vehicles/vehicle.schema.json) (generated from the compiler's own tables by
`scripts/gen_vehicle_schema.py`; a test keeps them identical). The compiler (`rocket_sim.vehicle.vehicle_file`) is the authority:
it also checks the physics a schema cannot (contiguity, symmetric tensors, triangle inequalities, positions inside the airframe).

Use it:

```bash
rocketsim vehicle vehicles/example_tvc_demo.json          # derived mass properties report (or --json)
rocketsim simulate configs/example_vehicle_sim.yaml       # a simulation config that says: vehicle_file: ../vehicles/example_tvc_demo.json
```

`vehicle_file:` and `rocket:` are mutually exclusive in a simulation config (one source of truth). The environment, launch,
numerics, sensors, estimator and controller still come from the simulation config.

## 1. Conventions (read once)

| item | convention |
|---|---|
| units | SI (m, kg, s, rad; angles in degrees only where a key name says `_deg`) |
| axial position `x` | metres **aft** of the **nose tip** (nose tip = 0, increasing toward the tail) |
| body axes | x forward (toward the nose), y right, z down (the same frame as the 6-DOF state, [frames.md](frames.md)) |
| lateral offset `offset_m: [y, z]` | the component CG's offset from the rocket axis in body y (right) and z (down) |
| reference point of mass properties | the instantaneous **centre of gravity** of the whole vehicle (computed, reported as `x_cg`, offset `(y, z)`) |
| inertia tensor `inertia_kgm2` | the 3x3 **tensor (matrix)** of a component about **its own CG**, in body axes: `[[Ixx, -Pxy, -Pxz], [-Pxy, Iyy, -Pyz], [-Pxz, -Pyz, Izz]]` with products of inertia `Pxy = integral x y dm` |
| sign convention | positive-definite diagonal, **off-diagonal entries are the NEGATIVES of the products**. CAD systems often print products: negate them when filling the matrix |
| vehicle tensor | `I_cg = sum_i [ I_i + m_i (|d_i|^2 E - d_i d_i^T) ]`, `d_i` = position of component i relative to the vehicle CG (parallel-axis theorem), body axes; returned by `MassProps.tensor()` and used unchanged by Euler's equations `I w_dot = M - w x (I w)` |
| axisymmetric fast path | when all lateral offsets and products are zero the code uses the (identical) scalar formulas; otherwise the full tensor path. Both are tested to agree |
| propellant | a uniform solid cylinder (the motor's diameter and length) at the motor centre; its mass follows the thrust curve at constant exhaust velocity. The CG therefore moves **forward** as it burns when the motor is aft of the structure CG |

## 2. Top level

```json
{
  "format": "rocket-sim-vehicle",
  "format_version": 1,
  "metadata": { "name": "...", "description": "...", "author": "...", "created": "...",
                "source": "manual | fusion360 | openrocket | ...",
                "data_quality": "design | measured | estimated | placeholder", "notes": "..." },
  "materials": { "fiberglass": 1850.0 },
  "geometry": { "reference_diameter_m": 0.0762 },
  "components": [ ... ],
  "motor": { "file": "../data/motors/AeroTech_K828FJ.eng", "designation": null, "aft_position_m": 1.60 },
  "parachutes": [ { "cd": 1.5, "diameter_m": 0.45, "trigger": "apogee", "delay_s": 1.0 } ],
  "aero": { "model": "barrowman", "surface_roughness_m": 6e-05, "provenance": { } }
}
```

* `metadata.data_quality` is carried into every flight record and dataset (`rocket.vehicle_source`): a `placeholder` vehicle is
  never confused with a weighed one.
* `motor.file` is a RASP `.eng` path relative to the vehicle file. `aft_position_m` is the nozzle exit plane (default: the aft end).
* `parachutes` and `aero` are passed through unchanged to the simulator's `rocket.parachutes` / `rocket.aero` sections
  (see [config_reference.md](config_reference.md); `aero.model` selects the model, `aero.provenance` declares the origin of imported
  coefficients, [aerodynamics.md](aerodynamics.md)).

## 3. Components

Every component has a unique `id`. Common optional fields: `name`, `notes`, `mass_kg`, `material`. `cg_x_m`, `offset_m` and `inertia_kgm2` are accepted only by components that carry them into the mass model (airframe sections and mass-bearing components); **fin sets and control-surface sets reject them** (their CG and inertia are derived from the planform; a key that would be ignored is an error, not silently dropped).

### Airframe (stacked from the nose tip; contiguous, diameters continuous)

| `type` | fields | notes |
|---|---|---|
| `nose` | `shape` (`ogive`, `cone`, `parabolic`, `elliptical`), `length_m`, `base_diameter_m`, `x_start_m` (0) | exactly one, first |
| `body_tube` | `length_m`, `diameter_m`, `x_start_m` | outer diameter |
| `transition` | `length_m`, `fore_diameter_m`, `aft_diameter_m`, `x_start_m` | `aft < fore` is a **boat-tail** (negative normal force, base-area drag) |

`x_start_m` may be omitted (file order stacks the sections); if given it must equal the end of the previous section within 1 um.
A forward diameter that does not match the previous aft diameter is an error. **Mass**: either `mass_kg`, or `material` +
`wall_thickness_m` (tube: exact annulus `pi (ro^2 - ri^2) L rho`; nose/transition: wetted shell area x thickness x density).
**Inertia**: if the component gives `inertia_kgm2` (and/or `offset_m`) its mass rides on a mass item at its CG with that tensor;
otherwise the simulator uses a **thin-shell estimate and says so** (`vehicle.notes`, run warnings).

### Fins and control surfaces

| `type` | fields |
|---|---|
| `fin_set` | `count`, `root_chord_m`, `tip_chord_m`, `span_m`, `sweep_m`, `thickness_m`, `x_root_le_m` (leading edge of the root chord) |
| `control_surface_set` | same, plus `roll_angle0_deg`, `max_deflection_deg`, `max_rate_deg_s`, `time_constant_s`, `delay_s` (movable fins/canards; the actuator model is built from these) |

Mass: `mass_kg` or `material` (plate mass `count x (root+tip)/2 x span x thickness x density`). One `fin_set` and one
`control_surface_set` are supported; more detailed fin geometry (airfoil sections, cant) is a future extension, not implemented.

### Mass-bearing components (no aerodynamic effect)

`motor_mount`, `avionics`, `battery`, `payload`, `recovery`, `ballast`, `structure`, `other`. Required: `mass_kg`, `cg_x_m`.
Inertia, in order of preference: `inertia_kgm2` (measured/CAD tensor about its own CG) -> `shape` + `dimensions_m` (uniform solid:
`box [length_x, width_y, height_z]` or `cylinder [length, diameter]`, `point` = zero) -> a **point mass with a warning**. Use
`offset_m` for off-axis placement (battery beside the avionics sled): it produces a lateral CG offset and products of inertia.

The motor (with its casing and propellant) is **not** a component; it comes from the `.eng` file so that the thrust curve, propellant
mass, casing mass and mass flow stay consistent with each other.

## 4. What the simulator derives (`rocketsim vehicle`)

Total / dry / propellant mass; component table; CG and full inertia tensor at ignition and burnout; CG travel (and static margin)
along the burn; CP and CN_alpha (subsonic, small angle); static margin; motor data (burn time, impulse, mean/peak propellant mass
flow); aerodynamic provenance; and the list of ESTIMATES the simulator had to make (thin-shell inertias, point masses).

## 4b. Validation performed by the compiler

Unique ids; exactly one nose; contiguous stack with continuous diameters; positive finite densities; wall thickness smaller than the smallest radius;
integer fin count >= 2; box/cylinder dimensions positive and of the right number; `offset_m` two finite numbers inside the airframe radius;
CG positions inside their component / the airframe; motor aft position inside the airframe; a tensor that is symmetric, has a non-negative
diagonal, satisfies the triangle inequalities **and is positive semi-definite**; `inertia_kgm2` and `shape` not both given. The input dictionary is never mutated.
**Not checked:** that masses are realistic, that a box fits inside the tube, interference between components, that the motor fits the mount (the builder checks motor vs body diameter and length).
**Mass-changing components:** only the motor propellant changes mass in flight (a per-component flag and a general mass-flow model are not implemented).

## 5. Where it comes from, and what is NOT claimed

* `placeholder`/`design`/`estimated` data are as good as the numbers entered. The simulator checks consistency, not truth.
* **Shell masses derived from density x thickness ignore adhesive, fillets, paint, bulkheads and hardware**: weigh the airframe, or
  give measured `mass_kg`.
* Thin-shell inertias of airframe sections are estimates (the axial inertia of a hollow tube is exact only for a thin wall).

## 6. Fusion 360 and OpenRocket (preparation, not implemented)

The format is the **target** of a future exporter/importer; no Fusion plugin and no `.ork` importer exist yet.

| Fusion 360 (Physical Properties, per body, relative to the chosen origin) | vehicle.json |
|---|---|
| Mass | `mass_kg` |
| Center of Mass (X, Y, Z) | `cg_x_m` (axial, from the nose tip, **aft positive**: convert from the CAD axis) and `offset_m` `[y, z]` |
| Moments of inertia at the center of mass (Ixx, Iyy, Izz) and Products (Ixy, Iyz, Izx) | `inertia_kgm2`, **re-expressed in body axes** (x nose-forward, y right, z down) and written as the matrix with negated products |
| body length / diameter / fin dimensions | the corresponding geometry fields |

Practical workflow today: set the CAD origin at the nose tip with the axis along the rocket, read each body's physical properties, and
type them into the JSON (the sign and axis conversion is the usual trap: check that `rocketsim vehicle` reproduces the CAD total mass
and CG).

| OpenRocket component | vehicle.json |
|---|---|
| Nose cone, Body tube, Transition | `nose`, `body_tube`, `transition` (shape, lengths, diameters, wall thickness, material density) |
| Trapezoidal fin set | `fin_set` (`x_root_le_m` = fin position from the nose tip) |
| Inner tube, centering ring, bulkhead, mass component | `structure` / `motor_mount` / `avionics` / `payload` ... with `mass_kg`, `cg_x_m` |
| Parachute / shock cord | `recovery` mass item + `parachutes` entry |
| Motor | `motor.file` (+ `aft_position_m`) |

Keeping all OpenRocket-specific knowledge in a converter leaves the physics free of its assumptions.

## 7. Evidence status

Implemented and covered by tests: compilation, validation rules, mass/CG/tensor against independent numpy summation, shell and plate
mass formulas, shape inertias, tensor sign convention, determinism, traceability, CG shift during the burn, and a RocketPy cross-check of
CN_alpha and CP for the example vehicle. **Not validated against any real weighed vehicle.**
