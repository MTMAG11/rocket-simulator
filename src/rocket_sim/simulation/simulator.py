"""The simulation loop: physics + environment + (sensors -> estimator -> controller ->
actuator) pipeline, with event handling and telemetry recording.

Per step (time t, true state y):

    1. evaluate forces/environment at (t, y)                     [physics, truth]
    2. sensors sample the TRUE state when due                    [truth -> measurement]
    3. estimator updates from MEASUREMENTS only                  [measurement -> estimate]
    4. controller (at its own rate) maps ESTIMATE -> command      [estimate -> command]
    5. actuator model turns the command into a gimbal angle      [command -> actual]
    6. phase machine, event bookkeeping, record row
    7. integrate t -> t + h with the actuator angle held fixed

The step length h is min(dt, time to the next scheduled discontinuity): thrust-curve corners,
parachute opening, sensor and controller ticks. Zero crossings (rail exit, apogee, ground
impact, altitude-triggered parachute) are located by bisection with re-integration, so event
times and impact velocity do not depend on interpolation of the output grid.
"""

from __future__ import annotations

import datetime as _dt
import heapq
import math
import time as _time
from typing import Any

import numpy as np

from ..config import SimConfig, config_hash, config_to_dict, resolve_path
from ..constants import G0
from ..control import Command, ControlInput, Controller, TVCActuator, build_controller
from ..control.actuators import ActuatorBank
from ..control.mixer import FinMixer
from ..data.schema import columns_for
from ..environment import ISAAtmosphere
from ..errors import SimulationError
from ..estimation import EstimatedState, TruthEstimator, TruthState, build_estimator
from ..physics.integrators import STEPPERS, refine_event
from ..physics.math3d import quat_to_euler
from ..sensors import SensorSuite
from ..version import PHYSICS_VERSION, SCHEMA_VERSION, SIM_VERSION
from .builder import (
    build_dynamics,
    build_environment,
    build_motor,
    build_vehicle,
    resolve_fidelity,
)
from .phases import PhaseInputs, PhaseMachine
from .record import (
    FlightEvent,
    FlightRecord,
    Recorder,
    RunMetadata,
    compute_summary,
)

MAX_SPEED_SANITY = 20_000.0  # [m/s] a run exceeding this is numerically diverged
_EPS = 1e-9


def _sha256_file(path) -> str:
    import hashlib

    return hashlib.sha256(path.read_bytes()).hexdigest()


class Simulation:
    """One flight. Construct, then ``run()`` -> FlightRecord. Reusable only by re-constructing
    (state is consumed by ``run``)."""

    def __init__(
        self,
        cfg: SimConfig,
        seed: int | None = None,
        simulation_id: str | None = None,
        controller: Controller | None = None,
    ) -> None:
        cfg.validate()
        self.input_cfg = cfg
        self.cfg, self.flags = resolve_fidelity(cfg)
        c = self.cfg
        self.seed = int(seed if seed is not None else (c.simulation.seed or 0))
        kids = np.random.SeedSequence(self.seed).spawn(8)

        self.motor = build_motor(c)
        self.input_files = {c.motor.file: _sha256_file(resolve_path(c, c.motor.file))}
        # id/hash cover the config AND the contents of external input files (a modified motor file is a different run)
        self.cfg_hash = config_hash({"config": config_to_dict(cfg), "input_files": self.input_files})
        self.simulation_id = simulation_id or f"{self.cfg_hash}-s{self.seed}"
        self.vehicle = build_vehicle(c, self.motor)
        self.env = build_environment(c, kids[0])
        self.dyn = build_dynamics(c, self.flags, self.vehicle, self.env)
        self.has_sensors = self.flags.sensors
        self.sensors = SensorSuite(c.sensors, kids[1:6]) if self.has_sensors else None
        self.estimator = (
            build_estimator(c.estimator, c.sensors, self.env.gravity, self.env.site_elevation)
            if (self.has_sensors and c.estimator.type != "none")
            or (c.estimator.type == "truth" and self.flags.dof == 6)
            else None
        )
        self.controller: Controller | None = None
        if self.flags.dof == 6 and (controller is not None or c.controller.type != "none"):
            self.controller = controller or build_controller(c.controller.type, c.controller.params)
        self.actuator = TVCActuator(c.tvc)
        self.fin_actuator: ActuatorBank | None = None
        self.mixer: FinMixer | None = None
        cfins = self.vehicle.aero.control_fins
        if cfins and self.flags.dof == 6:
            self.fin_actuator = ActuatorBank(
                [f.max_deflection for f in cfins],
                [f.max_rate for f in cfins],
                [f.time_constant for f in cfins],
                [f.delay for f in cfins],
            )
            self.mixer = FinMixer(cfins, self.vehicle.mass_props(0.0).x_cg)
        self.phases = PhaseMachine()
        self.warnings: list[str] = list(self.motor.sanity_warnings())
        if self.flags.dof == 6:
            self.warnings.extend(
                f"{n} (provide measured/CAD values)"
                for n in dict.fromkeys(self.vehicle.notes)
                if "ESTIMATE" in n
            )
        if self.flags.dof == 6:
            sm = self.vehicle.static_margin(0.0)
            if sm < 1.0:
                self.warnings.append(f"static margin at launch is {sm:.2f} cal (< 1 cal)")
            if sm <= 0.0:
                self.warnings.append("vehicle is statically UNSTABLE at launch (cp ahead of cg)")

    # ------------------------------------------------------------------------------------------
    def _design_data(self) -> dict:
        """DESIGN knowledge handed to a flight computer: the nominal gimbal-authority schedule against time since ignition, gravity,
        site elevation, launch axis and alignment time. Never the instantaneous true state."""
        veh, cfg = self.vehicle, self.cfg
        auth = self._controller_authority()
        t_ign, burn = veh.ignition_delay, veh.motor.times[-1]
        n = int(burn / 0.02) + 1
        table = []
        for k in range(n + 1):
            tau = min(k * 0.02, burn)
            v = auth(t_ign + tau)
            table.append([tau, None if v is None else float(v)])
        return {
            "authority_since_ignition": table,
            "gravity_m_s2": cfg.environment.gravity.g_ms2,
            "site_elevation_m": float(self.env.site_elevation),
            "launch_axis": [float(x) for x in self.dyn.rail_axis],
            "alignment_time_s": cfg.estimator.alignment_time_s,
            "magnetic_field_enu_t": list(cfg.sensors.magnetic_field_enu_t),
        }

    def _hil_init(self) -> dict | None:
        """Content of the protocol ``init`` message (None below fidelity 4, where there are no sensors)."""
        if not self.has_sensors:
            return None
        s = self.cfg.sensors
        specs = {}
        for name, attr in (
            ("accel", "accelerometer"),
            ("gyro", "gyroscope"),
            ("baro", "barometer"),
            ("gps", "gps"),
            ("mag", "magnetometer"),
        ):
            c = getattr(s, attr)
            if c.enabled:
                specs[name] = {"rate_hz": c.rate_hz, "noise_std": c.noise_std, "latency_s": c.latency_s}
        return {
            "controller_rate_hz": self.cfg.controller.rate_hz,
            "sensors": specs,
            "design": self._design_data(),
            "estimator": {"alignment_time_s": self.cfg.estimator.alignment_time_s},
        }

    def _fin_authority(self):
        """Command-per-angular-acceleration for control surfaces, from the (estimated) dynamic pressure.

        The flight computer is assumed to know the vehicle's mass properties and the standard atmosphere;
        it does NOT know the wind, so dynamic pressure comes from the estimated ground-relative speed."""
        veh, mixer = self.vehicle, self.mixer
        assert mixer is not None
        s_ref = veh.aero.ref_area
        isa = ISAAtmosphere()
        site = self.env.site_elevation
        vref = 15.0  # below this speed the surfaces have no useful authority

        def authority(t: float, inp) -> tuple[float, float, float] | None:
            vx, vy, vz = inp.velocity
            speed = math.sqrt(vx * vx + vy * vy + vz * vz)
            if speed < vref:
                return None
            rho = isa.at(site + inp.position[2]).density
            qs = 0.5 * rho * speed * speed * s_ref
            tau = inp.t_since_launch
            mp = veh.mass_props(
                veh.ignition_delay + (tau if tau is not None else 0.0)
            )  # indexed by time since launch
            gy, gz, gx = mixer.gain(1), mixer.gain(2), mixer.gain(0)
            if min(abs(gy), abs(gz)) < 1e-12:
                return None
            return (mp.iyy / (qs * gy), mp.iyy / (qs * gz), (mp.ixx / (qs * gx)) if abs(gx) > 1e-12 else 0.0)

        return authority

    def _authority_since_ignition(self):
        """Gimbal authority as a function of TIME SINCE (detected) LAUNCH: what a flight computer can index without the true clock."""
        auth = self._controller_authority()
        t_ign = self.vehicle.ignition_delay

        def authority(tau: float) -> float | None:
            return auth(t_ign + tau)

        return authority

    def _controller_authority(self):
        """Gimbal-per-angular-acceleration schedule I_yy / (T |lever|) known to the controller.

        The flight computer is assumed to know the nominal thrust curve (the config's thrust
        scale error is NOT known to it) and the airframe mass properties.
        """
        veh = self.vehicle
        scale = self.cfg.motor.thrust_scale
        knowledge = (
            self.cfg.controller.design_inertia_scale
        )  # the flight computer's belief about I/(T lever), relative to as-built
        peak = veh.motor.max_thrust / scale

        def authority(t: float) -> float | None:
            thrust = veh.thrust_at(t) / scale
            if thrust < 0.02 * peak:
                return None
            mp = veh.mass_props(t)
            lever = abs(mp.x_cg - veh.nozzle_x)
            return knowledge * mp.iyy / (thrust * lever)

        return authority

    # ------------------------------------------------------------------------------------------
    def run(self) -> FlightRecord:
        wall0 = _time.perf_counter()
        cfg, veh, env, dyn = self.cfg, self.vehicle, self.env, self.dyn
        dt_max = cfg.simulation.dt_s
        dt_descent = cfg.simulation.descent_dt_s or dt_max
        t_max = cfg.simulation.t_max_s
        step = STEPPERS[cfg.simulation.integrator]
        record_every = cfg.simulation.record_every
        terrain = env.terrain
        is6 = self.flags.dof == 6
        t_ign = veh.ignition_delay
        t_burn = veh.burnout_time
        site = env.site_elevation

        n_fins = len(self.vehicle.aero.control_fins) if is6 else 0
        columns = [c.name for c in columns_for(cfg.fidelity, self.estimator is not None, n_fins)]
        rec = Recorder(columns)
        events: list[FlightEvent] = []

        y = dyn.initial_state()
        t = 0.0
        prev_t = 0.0
        airborne = False
        apogee_done = False
        t_apogee = 0.0
        burnout_done = False
        landed = False
        ap_alt = 0.0
        chutes = veh.parachutes
        chute_scheduled = [False] * len(chutes)
        chute_logged = [False] * len(chutes)
        status = "ok"
        n_steps = 0
        row_counter = 0
        n_rows = 0
        last_est = EstimatedState()
        last_fin_cmd = (0.0, 0.0, 0.0)
        latest_readings = None
        isa = ISAAtmosphere()
        p_ref_baro: float | None = None
        h_ref_baro = 0.0

        # fixed discontinuities of the thrust curve (absolute time)
        breaks = [t_ign + tt for tt in veh.motor.times if t_ign + tt > _EPS]
        heapq.heapify(breaks)

        if self.controller is not None:
            self.controller.reset(
                {
                    "authority": self._controller_authority(),
                    "authority_since_ignition": self._authority_since_ignition(),
                    "fin_authority": self._fin_authority() if self.mixer is not None else None,
                    "launch_axis": dyn.rail_axis,
                    "design": self._design_data(),
                    "hil_init": self._hil_init(),
                }
            )
        ctrl_period = 1.0 / cfg.controller.rate_hz
        next_ctrl = 0.0
        src_cfg = cfg.controller.state_source
        use_truth_ctrl = (
            cfg.controller.use_truth
            or src_cfg == "truth"
            or isinstance(self.estimator, TruthEstimator)
            or (src_cfg == "auto" and (not self.has_sensors or self.estimator is None))
        )
        consumes_state = bool(getattr(self.controller, "consumes_state", True))
        ctrl_state_source = (
            "none"
            if self.controller is None
            else ("measurements_only" if not consumes_state else ("truth" if use_truth_ctrl else "estimate"))
        )
        if self.controller is not None and use_truth_ctrl and consumes_state:
            self.warnings.append(
                "controller is fed the TRUE state (use_truth / state_source=truth, or no sensors+estimator in the loop): "
                "results do not represent a flight computer working from measurements"
            )
        if self.controller is not None:
            self.flags.notes.append(f"controller_state_source={ctrl_state_source}")
        wants_samples = bool(getattr(self.controller, "wants_samples", False)) and self.sensors is not None
        if wants_samples:
            assert self.sensors is not None
            self.sensors.enable_collection()
        cmd_latency = cfg.controller.compute_time_s + cfg.controller.downlink_latency_s
        if hasattr(self.controller, "uplink_latency_s") and self.controller.uplink_latency_s is None:  # type: ignore[union-attr]
            self.controller.uplink_latency_s = cfg.controller.uplink_latency_s  # type: ignore[union-attr]

        def log_event(name: str, tt: float, yy: np.ndarray, **extra: float) -> None:
            vx, vy, vz = float(yy[3]), float(yy[4]), float(yy[5])
            info = {
                "altitude": float(yy[2] - terrain.height(float(yy[0]), float(yy[1]))),
                "speed": math.sqrt(vx * vx + vy * vy + vz * vz),
                "vz": vz,
                "x": float(yy[0]),
                "y": float(yy[1]),
            }
            info.update(extra)
            events.append(FlightEvent(tt, name, info))

        def make_row(tt: float, yy: np.ndarray, ev, dt_row: float) -> dict[str, float]:
            q = dyn.quaternion(yy, tt)
            hd, pitch, roll = quat_to_euler(q)
            om = dyn.omega(yy)
            vx, vy, vz = float(yy[3]), float(yy[4]), float(yy[5])
            mpr = max(ev.mass - veh.mass_model.static_mass, 0.0)
            row: dict[str, float] = {
                "t": tt,
                "dt": dt_row,
                "phase": float(int(self.phases.phase)),
                "pos_x": float(yy[0]),
                "pos_y": float(yy[1]),
                "pos_z": float(yy[2]),
                "vel_x": vx,
                "vel_y": vy,
                "vel_z": vz,
                "acc_x": ev.a[0],
                "acc_y": ev.a[1],
                "acc_z": ev.a[2],
                "quat_w": q[0],
                "quat_x": q[1],
                "quat_y": q[2],
                "quat_z": q[3],
                "roll": roll,
                "pitch": pitch,
                "yaw": hd,
                "omega_p": om[0],
                "omega_q": om[1],
                "omega_r": om[2],
                "alpha_p": ev.wdot[0],
                "alpha_q": ev.wdot[1],
                "alpha_r": ev.wdot[2],
                "mass": ev.mass,
                "prop_mass": mpr,
                "dry_mass": veh.mass_model.static_mass,
                "cg": ev.x_cg,
                "cp": ev.x_cp,
                "static_margin": ev.static_margin,
                "ixx": ev.ixx,
                "iyy": ev.iyy,
                "altitude": float(yy[2] - terrain.height(float(yy[0]), float(yy[1]))),
                "altitude_msl": site + float(yy[2]),
                "temperature": ev.temp,
                "pressure": ev.pres,
                "density": ev.rho,
                "speed_of_sound": ev.sos,
                "wind_x": ev.wind[0],
                "wind_y": ev.wind[1],
                "wind_z": ev.wind[2],
                "airspeed": ev.airspeed,
                "speed": math.sqrt(vx * vx + vy * vy + vz * vz),
                "gravity": ev.g,
                "thrust": ev.thrust,
                "drag": ev.drag,
                "lift": ev.lift,
                "chute_drag": ev.chute_drag,
                "mach": ev.mach,
                "aoa": ev.alpha,
                "sideslip": ev.beta,
                "qdyn": ev.qdyn,
            }
            if is6:
                mp6 = veh.mass_props(tt)
                row.update(
                    {
                        "izz": mp6.izz_eff,
                        "ixy": mp6.ixy,
                        "ixz": mp6.ixz,
                        "iyz": mp6.iyz,
                        "cg_y": mp6.y_cg,
                        "cg_z": mp6.z_cg,
                        "fsp_x": ev.spec_force[0],
                        "fsp_y": ev.spec_force[1],
                        "fsp_z": ev.spec_force[2],
                    }
                )
            if cfg.fidelity >= 3:
                row["tvc_cmd_y"], row["tvc_cmd_z"] = self.actuator.cmd
                row["tvc_y"], row["tvc_z"] = self.actuator.state
            if self.fin_actuator is not None:
                row["fin_cmd_roll"], row["fin_cmd_pitch"], row["fin_cmd_yaw"] = last_fin_cmd
                for i_, d_ in enumerate(self.fin_actuator.state):
                    row[f"fin_{i_}"] = d_
                    row[f"fin_dcmd_{i_}"] = self.fin_actuator.cmd[i_]
            if self.has_sensors:
                row.update(sensor_cols)
            if self.estimator is not None:
                row.update(est_cols(last_est))
            return row

        sensor_cols: dict[str, float] = {}

        def est_cols(e: EstimatedState) -> dict[str, float]:
            return {
                "est_valid": float(e.valid),
                "launch_detected": float(e.launch_detected),
                "est_pos_x": e.position[0],
                "est_pos_y": e.position[1],
                "est_pos_z": e.position[2],
                "est_vel_x": e.velocity[0],
                "est_vel_y": e.velocity[1],
                "est_vel_z": e.velocity[2],
                "est_quat_w": e.quaternion[0],
                "est_quat_x": e.quaternion[1],
                "est_quat_y": e.quaternion[2],
                "est_quat_z": e.quaternion[3],
                "est_gyro_bias_x": e.extra.get("gyro_bias_x", 0.0),
                "est_gyro_bias_y": e.extra.get("gyro_bias_y", 0.0),
                "est_gyro_bias_z": e.extra.get("gyro_bias_z", 0.0),
            }

        while True:
            ev = dyn.evaluate(t, y)

            # ---- sensors -> estimator -> controller -> actuator command --------------------
            if self.sensors is not None:
                rd = latest_readings = self.sensors.update(
                    t,
                    ev.spec_force,
                    dyn.omega(y),
                    ev.pres,
                    dyn.position(y),
                    dyn.velocity(y),
                    dyn.quaternion(y, t),
                )
                if rd.baro_new and p_ref_baro is None:
                    p_ref_baro = rd.baro_pressure
                    h_ref_baro = isa.pressure_to_altitude(p_ref_baro)
                baro_alt = 0.0
                if p_ref_baro is not None:
                    baro_alt = isa.pressure_to_altitude(max(rd.baro_pressure, 1.0)) - h_ref_baro
                sensor_cols = {
                    "meas_accel_x": rd.accel[0],
                    "meas_accel_y": rd.accel[1],
                    "meas_accel_z": rd.accel[2],
                    "meas_accel_new": float(rd.accel_new),
                    "meas_gyro_new": float(rd.gyro_new),
                    "meas_mag_new": float(rd.mag_new),
                    "meas_gyro_x": rd.gyro[0],
                    "meas_gyro_y": rd.gyro[1],
                    "meas_gyro_z": rd.gyro[2],
                    "meas_baro_pressure": rd.baro_pressure,
                    "meas_baro_altitude": baro_alt,
                    "meas_baro_new": float(rd.baro_new),
                    "meas_gps_pos_x": rd.gps[0],
                    "meas_gps_pos_y": rd.gps[1],
                    "meas_gps_pos_z": rd.gps[2],
                    "meas_gps_vel_x": rd.gps[3],
                    "meas_gps_vel_y": rd.gps[4],
                    "meas_gps_vel_z": rd.gps[5],
                    "meas_gps_new": float(rd.gps_new),
                    "meas_mag_x": rd.mag[0],
                    "meas_mag_y": rd.mag[1],
                    "meas_mag_z": rd.mag[2],
                }
            if self.estimator is not None:
                truth = None
                if isinstance(self.estimator, TruthEstimator):
                    truth = TruthState(
                        dyn.position(y),
                        dyn.velocity(y),
                        dyn.quaternion(y, t),
                        dyn.omega(y),
                        t_ign if t >= t_ign else None,
                    )
                last_est = self.estimator.update(t, latest_readings, truth)
            if self.controller is not None and t >= next_ctrl - _EPS:
                if (
                    not consumes_state
                ):  # e.g. a HIL bridge: the controller sees SENSOR SAMPLES only, never a state
                    inp = ControlInput(
                        t,
                        None,
                        False,
                        (0.0, 0.0, 0.0),
                        (0.0, 0.0, 0.0),
                        (1.0, 0.0, 0.0, 0.0),
                        (0.0, 0.0, 0.0),
                        0,
                        latest_readings,  # raw MEASUREMENTS are allowed; no state is
                        self.sensors.drain() if wants_samples and self.sensors is not None else [],
                        "none",
                    )
                elif use_truth_ctrl:
                    ts = (t - t_ign) if t >= t_ign else None
                    inp = ControlInput(
                        t,
                        ts,
                        True,
                        dyn.position(y),
                        dyn.velocity(y),
                        dyn.quaternion(y, t),
                        dyn.omega(y),
                        int(self.phases.phase),
                        latest_readings,
                        self.sensors.drain() if wants_samples and self.sensors is not None else [],
                        "truth",
                    )
                else:
                    ts = (t - last_est.launch_time) if last_est.launch_time is not None else None
                    inp = ControlInput(
                        t,
                        ts,
                        last_est.valid,
                        last_est.position,
                        last_est.velocity,
                        last_est.quaternion,
                        last_est.omega,
                        2
                        if last_est.launch_detected
                        else 0,  # estimated: 0 = on the pad, 2 = launch detected (no true phase)
                        latest_readings,
                        self.sensors.drain() if wants_samples and self.sensors is not None else [],
                        "estimate",
                    )
                cmd = self.controller.update(inp)
                if not (math.isfinite(cmd.tvc_y) and math.isfinite(cmd.tvc_z)):
                    raise SimulationError(f"controller returned a non-finite command at t={t:.3f}")
                # the command reaches the actuator after the flight computer's compute time and the downlink latency
                self.actuator.command(t + cmd_latency, Command(cmd.tvc_y, cmd.tvc_z))
                if self.fin_actuator is not None and self.mixer is not None:
                    if not all(math.isfinite(v) for v in (cmd.fin_roll, cmd.fin_pitch, cmd.fin_yaw)):
                        raise SimulationError(f"controller returned a non-finite fin command at t={t:.3f}")
                    self.fin_actuator.command(t + cmd_latency, self.mixer.mix(cmd))
                    last_fin_cmd = (cmd.fin_roll, cmd.fin_pitch, cmd.fin_yaw)
                while next_ctrl <= t + _EPS:
                    next_ctrl += ctrl_period

            # ---- phase machine ------------------------------------------------------------
            if not burnout_done and t >= t_burn - _EPS:
                burnout_done = True
                log_event("burnout", t, y)
            # (the first row is always PRELAUNCH, even when ignition is commanded at t = 0)
            new_phase = (
                None
                if n_rows == 0
                else self.phases.update(
                    PhaseInputs(t, t >= t_ign - _EPS, airborne, burnout_done, apogee_done, landed)
                )
            )
            if new_phase is not None:
                events.append(FlightEvent(t, "phase_change", {"phase": float(int(new_phase))}))

            # chute trigger bookkeeping
            for ci, t_start in enumerate(dyn.chute_starts):
                if t_start is not None and not chute_logged[ci] and t >= t_start - _EPS:
                    chute_logged[ci] = True
                    log_event("parachute_deploy", t, y, index=float(ci))

            if row_counter % record_every == 0 or landed or (events and events[-1].t == t):
                rec.append(make_row(t, y, ev, t - prev_t))
            row_counter += 1
            n_rows += 1
            nxt_phase = self.phases.advance_transient()
            if nxt_phase is not None:
                events.append(FlightEvent(t, "phase_change", {"phase": float(int(nxt_phase))}))

            if landed:
                break
            if (
                apogee_done
                and cfg.simulation.stop_after_apogee_s is not None
                and t >= t_apogee + cfg.simulation.stop_after_apogee_s - _EPS
            ):
                status = "truncated"
                break
            if t >= t_max - _EPS:
                status = "timeout"
                self.warnings.append(f"reached t_max={t_max} s without landing")
                break
            if not airborne and burnout_done and t > t_burn + 2.0:
                status = "no_liftoff"
                self.warnings.append("vehicle never left the pad (thrust-to-weight < 1?)")
                break

            # ---- choose the step ---------------------------------------------------------------
            h = min(dt_descent if apogee_done else dt_max, t_max - t)
            while breaks and breaks[0] <= t + _EPS:
                heapq.heappop(breaks)
            nxt = breaks[0] if breaks else math.inf
            if self.sensors is not None:
                nxt = min(nxt, self.sensors.next_event_time())
            if self.controller is not None:
                nxt = min(nxt, next_ctrl)
            if nxt - t > _EPS:
                h = min(h, nxt - t)
            h = max(h, 1e-9)

            if is6:
                dyn.controls.tvc_y, dyn.controls.tvc_z = self.actuator.state
                if self.fin_actuator is not None:
                    dyn.controls.fin = list(self.fin_actuator.state)  # ACTUAL fin positions
            f = dyn.derivative
            y_new = dyn.post_step(step(f, t, y, h))
            n_steps += 1
            if not np.all(np.isfinite(y_new)):
                raise SimulationError(f"non-finite state at t={t:.4f} s (step {n_steps})")

            # ---- zero-crossing events -----------------------------------------------------------
            t_new = t + h
            candidates: list[tuple[float, str, np.ndarray]] = []

            def locate(name: str, g, g_old: float, g_new: float, rising: bool) -> None:
                crossed = (g_old < 0.0 <= g_new) if rising else (g_old > 0.0 >= g_new)
                if crossed:
                    te, ye = refine_event(step, f, g, t, y, h)
                    candidates.append((te, name, ye))

            if dyn.on_rail:
                axis_len = cfg.launch.rail_length_m + 1e-4

                def g_rail(tt: float, yy: np.ndarray) -> float:
                    return dyn.rail_distance(yy) - axis_len

                locate("rail_exit", g_rail, g_rail(t, y), g_rail(t_new, y_new), True)
            if airborne:

                def g_ground(tt: float, yy: np.ndarray) -> float:
                    return float(yy[2]) - terrain.height(float(yy[0]), float(yy[1]))

                locate("landing", g_ground, g_ground(t, y), g_ground(t_new, y_new), False)
                if not apogee_done:

                    def g_vz(tt: float, yy: np.ndarray) -> float:
                        return float(yy[5])

                    locate("apogee", g_vz, g_vz(t, y), g_vz(t_new, y_new), False)
                if float(y[5]) < 0.0:
                    for ci, ch in enumerate(chutes):
                        if ch.trigger == "altitude" and not chute_scheduled[ci]:

                            def g_chute(tt: float, yy: np.ndarray, alt=ch.altitude) -> float:
                                return float(yy[2]) - terrain.height(float(yy[0]), float(yy[1])) - alt

                            locate(
                                f"chute_altitude:{ci}", g_chute, g_chute(t, y), g_chute(t_new, y_new), False
                            )

            if candidates:
                te, name, ye = min(candidates, key=lambda c: c[0])
                t_new, y_new = te, dyn.post_step(ye)  # event state must obey the same invariants (|q| = 1)
            else:
                name = ""

            prev_t = t
            t, y = t_new, y_new
            if is6:
                self.actuator.step(prev_t, t - prev_t)
                if self.fin_actuator is not None:
                    self.fin_actuator.step(prev_t, t - prev_t)
                # controls seen by the force evaluation at the start of the next loop (and by the logged row) are the
                # actuator state AT THIS time, not the previous step's
                dyn.controls.tvc_y, dyn.controls.tvc_z = self.actuator.state
                if self.fin_actuator is not None:
                    dyn.controls.fin = list(self.fin_actuator.state)

            if name == "rail_exit":
                dyn.on_rail = False
                log_event("rail_exit", t, y)
            elif name == "apogee":
                apogee_done = True
                t_apogee = t
                ap_alt = float(y[2])
                log_event("apogee", t, y)
                for ci, ch in enumerate(chutes):
                    if ch.trigger == "apogee" and not chute_scheduled[ci]:
                        chute_scheduled[ci] = True
                        self._schedule_chute(ci, t + ch.delay, breaks)
            elif name.startswith("chute_altitude:"):
                ci = int(name.split(":")[1])
                chute_scheduled[ci] = True
                self._schedule_chute(ci, t + chutes[ci].delay, breaks)
            elif name == "landing":
                landed = True
                y[2] = terrain.height(float(y[0]), float(y[1]))
                log_event("landing", t, y)

            if not airborne and dyn.rail_distance(y) >= self.cfg.phases.liftoff_distance_m:
                airborne = True
                log_event("liftoff", t, y)

        wall = _time.perf_counter() - wall0
        data = rec.finish()
        prov = self.vehicle.aero.provenance
        if prov is not None and data.shape[0] > 3:
            cidx = {c: i for i, c in enumerate(columns)}
            asc = (data[:, cidx["phase"]] < 5) & (
                data[:, cidx["airspeed"]] > 20.0
            )  # before apogee, flying (not the pad)
            if (
                asc.any()
                and prov.mach_range is not None
                and float(data[asc, cidx["mach"]].max()) > prov.mach_range[1] * 1.0001
            ):
                self.warnings.append(
                    f"flight exceeded the stated Mach range of the aerodynamic model ({prov.model}: Mach {prov.mach_range[0]:g}-{prov.mach_range[1]:g}, "
                    f"reached {float(data[asc, cidx['mach']].max()):.2f}); the coefficients are extrapolated, see docs/aerodynamics.md"
                )
            if asc.any() and prov.alpha_range_deg is not None and "aoa" in cidx:
                amax = math.degrees(float(data[asc, cidx["aoa"]].max()))
                if amax > prov.alpha_range_deg[1] * 1.0001:
                    self.warnings.append(
                        f"angle of attack reached {amax:.0f} deg before apogee, beyond the {prov.alpha_range_deg[1]:g} deg over which the "
                        f"{prov.model} model is a linear-theory estimate (larger angles use the crossflow estimate)"
                    )
        meta = RunMetadata(
            simulation_id=self.simulation_id,
            seed=self.seed,
            config_hash=self.cfg_hash,
            sim_version=SIM_VERSION,
            physics_version=PHYSICS_VERSION,
            schema_version=SCHEMA_VERSION,
            fidelity=self.cfg.fidelity,
            fast=self.cfg.fast,
            dt=dt_max,
            integrator=self.cfg.simulation.integrator,
            created_utc=_dt.datetime.now(_dt.UTC).isoformat(timespec="seconds"),
            status=status,
            warnings=list(self.warnings),
            notes=list(self.flags.notes),
            config=config_to_dict(self.input_cfg),
            wall_time_s=wall,
            input_files=self.input_files,
            aero_provenance=(self.vehicle.aero.provenance.to_dict() if self.vehicle.aero.provenance else {}),
            controller_state_source=ctrl_state_source,
            n_steps=n_steps,
        )
        record = FlightRecord(columns, data, events, meta)
        lz = self.cfg.landing_zone
        record.summary = compute_summary(
            record, G0, None if lz is None else (lz.center_m[0], lz.center_m[1], lz.radius_m)
        )
        record.summary["apogee_event_altitude_m"] = ap_alt if apogee_done else None
        return record

    def _schedule_chute(self, index: int, start: float, breaks: list[float]) -> None:
        """Arm parachute ``index``: drag begins at ``start`` and ramps over its inflation time."""
        self.dyn.chute_starts[index] = start
        heapq.heappush(breaks, start)
        inflation = self.vehicle.parachutes[index].inflation_time
        if inflation > 0:
            heapq.heappush(breaks, start + inflation)


def run_simulation(
    cfg: SimConfig, seed: int | None = None, simulation_id: str | None = None, controller: Any = None
) -> FlightRecord:
    """Convenience wrapper: build and run one simulation."""
    return Simulation(cfg, seed, simulation_id, controller).run()
