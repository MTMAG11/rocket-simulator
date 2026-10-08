# Aerodynamics (V1.1 model hierarchy)

All aerodynamic models implement one interface (`vehicle/aero.py::AerodynamicModel`) and are selected with
`rocket.aero.model`. They are *coefficient providers*; the equations of motion never know which one is in use.

## 1. Hierarchy

| `aero.model` | what it is | needs | use it for |
|---|---|---|---|
| `simplified` | one flat Cd (user value) + Barrowman CNa/CP from the geometry | `aero.cd` | quick studies, when only a drag number is known |
| `barrowman` (alias `buildup`) | Barrowman normal force/CP (Mach-independent) + component drag build-up (Re, Mach, roughness, base, boat-tail, transitions). **Default; the V1 model, generalised** | geometry | the baseline (apogee agreement with 7 flights only; [validation.md](validation.md)) |
| `enhanced` | `barrowman` + Mach-dependent fin lift slope and fin CP, fin stall, transonic/supersonic fin wave drag | geometry, `aero.stall_angle_deg` | supersonic or high-alpha flight; **not flight-validated** |
| `table` | Cd(Mach) lookup (coast and powered) + geometry-derived (or supplied) CNa/CP | `aero.cd_table` | a team's RASAero / wind-tunnel curve |
| `table2d` | full CD/CL/CM over Mach x alpha from a CSV (`mach,alpha_deg,cd,cl[,cm]`, complete grid, bilinear) | `aero.table2d_file` | **the CFD / wind-tunnel interface** (future CFD plugs in here) |
| `constant` | fixed numbers | `aero.cd` | tests, analysis |

A CFD-derived model needs no code change: export a grid, point `table2d_file` at it. The loader checks the grid is
complete and that Mach/alpha are strictly increasing. `Cm` columns are referred to `aero.x_cm_ref_from_nose_m`.

## 2. Coefficient interface

* `coefficients(mach, reynolds, powered) -> (cd0, cn_alpha, x_cp)` - the small-angle data.
* `force_coefficients(mach, alpha, reynolds, powered) -> (CA, CN, x_cp_force, x_cp_static)` - body-axis axial and normal
  coefficients valid for the whole angle-of-attack range 0..pi.
* Wind-axis quantities are derived consistently, for any model:

```
CD(M, a, Re) = CA cos a + CN sin a          (along -v_rel)
CL(M, a, Re) = CN cos a - CA sin a          (normal to v_rel, in the alpha plane)
Cm(M, a, Re; x_ref) = -CN (x_cp - x_ref) / d_ref
```

Referenced to `S = pi d_ref^2/4`, `d_ref` = largest body diameter (`rocket.reference_diameter_m` overrides).
Dependencies: **Mach** (skin-friction compressibility factor, base-drag blend 0.8-1.0, nose wave drag 0.8-1.2, enhanced
fin lift and wave drag), **angle of attack** (CN = CNa sin a |cos a| + Cd_cf (A_planform/S) sin^2 a; fin stall in
`enhanced`; the table models use their grids), **Reynolds** (skin friction only: laminar below 5e5, Schlichting
turbulent above, roughness cut-off), **geometry** (areas, fineness, boat-tail angle, transitions, fin planform).

## 3. Barrowman model (default)

Normal force and CP are Barrowman (1966/67): nose `CNa = 2`, CP 0.466 L (ogive); transition
`CNa = 2[(d_aft/d_ref)^2 - (d_fore/d_ref)^2]` (negative for a boat-tail) with Barrowman's CP; fins with `K_fb = 1 + R/(s+R)`.
The telescoping invariant (`sum of section CNa = 2[(d_last/d_ref)^2 - 0]` for a body that ends at `d_last`) is a unit test.
The drag build-up is the V1 build-up (Hoerner form factor and base drag, Raymer roughness cut-off) generalised: wetted
area from the section profile, a boat-tail adds a pressure-recovery term and an attached-flow fraction (<= 8 deg attached,
>= 15 deg separated, linear between: a *rule of thumb* from Hoerner), an expansion step adds a base-like term. It reproduces
the V1 numbers exactly for single-diameter airframes; the only physics difference is that crossflow drag now acts at the
planform centroid (golden numbers changed by 0.01 %, physics 1.2.0).

## 4. Enhanced model

Fin lift slope vs Mach: Helmbold/Diederich (subsonic) -> Ackeret (supersonic), blended through M 0.8-1.2 with a smoothstep;
the fin CP moves toward the planform centroid supersonically; fin lift saturates through a `tanh` stall near
`stall_angle_deg` (default 18 deg); fin wave drag (Ackeret) `4 (t/c)^2 / sqrt(M^2-1) x (fin planform / S)` is switched on across M 0.9-1.2. Tests: the slope ratio equals
the closed forms at M = 0, 0.6 and 1.6; the supersonic CP shifts aft; stall bounds the force. **Transonic accuracy is
+-30-50 % by construction (blend of two linear-theory limits).** In every one of the seven reconstructed flights the
`enhanced` apogee equals the Barrowman apogee to <= 0.4 m ([error_budget.md](error_budget.md)): none of them exercised
the transonic/supersonic fin behaviour, so this model has **no real-flight validation**.

## 5. Geometry tests (alpha / relative wind)

The angle-of-attack and sideslip geometry is verified independently of the simulator (`tests/test_aero_v11.py`): five
relative-velocity cases with wind (including reversed and sideways flow); at alpha = 0 the aerodynamic force is exactly
along the relative wind; the normal-force moment about the CG has the restoring sign for CP behind CG and the correct
magnitude; drag equals `q S CD` in the wind axes; `CL(alpha)` and `CD(alpha)` obey the small-angle limits and are symmetric
about 90 deg for a symmetric body. Reynolds trends: smooth finless airframes get lower Cf at higher Re until the roughness
cut-off. Mach: skin-friction, base and wave components each tested.

## 5b. Provenance of every coefficient (V1.2)

Every model carries an `AeroProvenance` (`vehicle.aero.provenance`), stored in each flight record (`meta.aero_provenance`) and as
`aero_provenance_kind` / `aero_model` in dataset `runs.parquet`. It states, **separately for drag, normal force / CP and damping**, the
`kind` (`analytical`, `barrowman`, `empirical`, `imported_table`, `experimental`, `cfd`, `user_defined`), the source, a stated confidence
(low / medium / high / unknown: a judgement, not computed), the applicable Mach and angle-of-attack ranges, and **how Reynolds
number enters**:

| model | drag | normal force / CP | damping | Reynolds |
|---|---|---|---|---|
| `barrowman` | empirical build-up (medium; stated Mach 0-0.9) | Barrowman (medium) | strip theory (low) | skin friction only |
| `enhanced` | build-up + Ackeret fin wave drag (**low**; "no real-flight validation") | Barrowman | strip theory | skin friction only |
| `simplified`, `constant` | user's number | Barrowman (or supplied) | strip theory | none |
| `table`, `table2d` | **user-declared** via `aero.provenance: {kind, source, confidence}`; undeclared = `user_defined`, source "no declared provenance" | Barrowman unless supplied | strip theory | none (no Re axis in the data) |

The dataset label `aero_provenance_kind` is `estimate` (everything estimated here), `mixed` (some supplied, some estimated) or
`imported` (currently unreachable because damping is always estimated). A declared `experimental`/`cfd` drag table therefore still gives
`mixed` and never reads as "measured". Tests check that the declared Reynolds dependence matches actual behaviour (Cd changes with Re for
the build-up only; CNa and CP never; table/constant models are exactly Re-independent).

**Ranges are enforced as warnings.** After every run the simulator compares the Mach number and angle of attack reached before apogee with the model's stated range
(barrowman: Mach 0-0.9 and 15 degrees of linear-theory angle of attack; enhanced: Mach 0-2) and adds a run warning if they were exceeded. The ranges are
judgements, not validated bounds, and a user-declared `experimental`/`cfd` kind is a statement that is not checked.

**Large-angle note.** The crossflow force acts at the planform centroid, which can be ahead of the CG, so at larger angles of attack
(and in the enhanced model at high Mach, where fin lift falls) the pitching moment can turn destabilising even with a positive static
margin: the coefficient tests assert restoring moments only up to 10 degrees.

## 6. Accuracy summary

| regime | status |
|---|---|
| subsonic (M < 0.8), small alpha, standard airframes | component build-up good to ~10-15 % in Cd (prior); apogee of 7 flights -6.4..+13.4 % with unmeasured inputs ([validation.md](validation.md)) |
| transonic/supersonic | analytic models only; unvalidated; unexplained Erebus 11 burnout-speed gap |
| large alpha (> 20 deg), tumbling | crossflow model (Hoerner), qualitatively right, not validated |
| below ~5 kg (small rockets) | no validation data; high sensitivity to fin thickness/roughness |
| not modelled | fin flutter, protuberances (use `extra_cd`), plume interaction, fin cant, Reynolds effects on pressure drag |
