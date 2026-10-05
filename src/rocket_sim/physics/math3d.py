"""Quaternion and rotation utilities. Pure-Python scalar math (fast for 3-vectors).

Frames and conventions (see docs/frames.md):

* Launch frame L : East-North-Up (x=E, y=N, z=U), right-handed, origin at the pad. It is
  treated as inertial (flat, non-rotating Earth).
* Body frame B   : x toward the nose, y to the "right", z completes the right-handed set
  (aircraft FRD: when level and heading north, y=East and z=Down).
* Quaternions q = (w, x, y, z), Hamilton convention, unit norm, representing the rotation
  that maps body-frame vectors into the launch frame: v_L = q (x) v_B (x) q*.
* Euler angles (display only) follow the aerospace 3-2-1 sequence relative to local NED:
  heading (yaw, clockwise from North), pitch (nose above horizon), roll about the nose axis.
  They are singular at pitch = +-90 deg, which is why the state uses quaternions.
"""

from __future__ import annotations

import math

Vec = tuple[float, float, float]
Quat = tuple[float, float, float, float]


def norm(v: Vec) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def dot(a: Vec, b: Vec) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec, b: Vec) -> Vec:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def quat_mul(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_conj(q: Quat) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def quat_normalize(q: Quat) -> Quat:
    n = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
    if n == 0.0:
        raise ValueError("cannot normalise a zero quaternion")
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_rotate(q: Quat, v: Vec) -> Vec:
    """Rotate body-frame vector v into the launch frame (v_L = R(q) v_B)."""
    w, x, y, z = q
    # v' = v + 2w (u x v) + 2 u x (u x v) with u = (x, y, z)
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    )


def quat_rotate_inv(q: Quat, v: Vec) -> Vec:
    """Rotate launch-frame vector v into the body frame (v_B = R(q)^T v_L)."""
    return quat_rotate((q[0], -q[1], -q[2], -q[3]), v)


def quat_to_dcm(q: Quat) -> tuple[Vec, Vec, Vec]:
    """Rows of the body->launch rotation matrix R (v_L = R v_B)."""
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def dcm_to_quat(r: tuple[Vec, Vec, Vec]) -> Quat:
    """Convert a body->launch rotation matrix to a unit quaternion (Shepperd's method)."""
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = (0.25 * s, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s)
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        q = ((r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s)
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        q = ((r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s)
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        q = ((r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s)
    return quat_normalize(q)


def quat_from_pointing(elevation: float, azimuth: float, roll: float = 0.0) -> Quat:
    """Attitude whose nose axis points at (elevation above horizon, azimuth clockwise from N).

    Angles in radians. ``roll`` rotates about the nose axis (right-hand rule about +x_B).
    The reference "right" axis is horizontal and 90 deg clockwise from the heading, so a
    level rocket pointing North has y_B=East, z_B=Down (FRD).
    """
    ce, se = math.cos(elevation), math.sin(elevation)
    ca, sa = math.cos(azimuth), math.sin(azimuth)
    xb = (ce * sa, ce * ca, se)  # nose, ENU
    yb0 = (ca, -sa, 0.0)  # right, ENU
    zb0 = cross(xb, yb0)
    cr, sr = math.cos(roll), math.sin(roll)
    yb = (cr * yb0[0] + sr * zb0[0], cr * yb0[1] + sr * zb0[1], cr * yb0[2] + sr * zb0[2])
    zb = cross(xb, yb)
    # columns of R are the body axes expressed in launch coordinates
    rows = (
        (xb[0], yb[0], zb[0]),
        (xb[1], yb[1], zb[1]),
        (xb[2], yb[2], zb[2]),
    )
    return dcm_to_quat(rows)


def quat_to_euler(q: Quat) -> tuple[float, float, float]:
    """(heading, pitch, roll) in radians: heading clockwise from North, pitch above horizon.

    Display only; singular at pitch = +-90 deg (roll and heading become coupled; at exactly
    vertical heading is returned as 0 by convention of atan2(0,0)).
    """
    r = quat_to_dcm(q)
    # body axes in NED: N = ENU.y, E = ENU.x, D = -ENU.z
    y_d = -r[2][1]
    z_d = -r[2][2]
    pitch = math.asin(max(-1.0, min(1.0, r[2][0])))  # = asin(x_up)
    heading = math.atan2(r[0][0], r[1][0])  # atan2(E, N) of the nose axis
    roll = math.atan2(y_d, z_d)
    return heading, pitch, roll


def quat_from_axis_angle(axis: Vec, angle: float) -> Quat:
    n = norm(axis)
    if n == 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    s = math.sin(angle / 2.0) / n
    return (math.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s)


def quat_error_vector(q_target: Quat, q_actual: Quat) -> Vec:
    """Small-rotation vector (body frame of q_actual) taking actual toward target.

    e_B = 2 * sign(w) * vec( q_actual* (x) q_target ) -- exact for small errors and
    well-behaved for large ones (shortest-path).
    """
    qe = quat_mul(quat_conj(q_actual), q_target)
    s = 1.0 if qe[0] >= 0.0 else -1.0
    return (2.0 * s * qe[1], 2.0 * s * qe[2], 2.0 * s * qe[3])


def angle_between(a: Vec, b: Vec) -> float:
    na, nb = norm(a), norm(b)
    if na == 0.0 or nb == 0.0:
        return 0.0
    c = max(-1.0, min(1.0, dot(a, b) / (na * nb)))
    return math.acos(c)
