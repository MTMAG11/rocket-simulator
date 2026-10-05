import math

import numpy as np
import pytest

from rocket_sim.physics.math3d import (
    angle_between,
    dcm_to_quat,
    quat_conj,
    quat_error_vector,
    quat_from_axis_angle,
    quat_from_pointing,
    quat_mul,
    quat_rotate,
    quat_rotate_inv,
    quat_to_dcm,
    quat_to_euler,
)

rng = np.random.default_rng(7)


def rand_quat():
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    return tuple(float(x) for x in q)


def test_rotation_matches_dcm_and_inverse():
    for _ in range(50):
        q = rand_quat()
        v = tuple(float(x) for x in rng.normal(size=3))
        r = np.array(quat_to_dcm(q))
        assert np.allclose(r @ np.array(v), quat_rotate(q, v), atol=1e-12)
        assert np.allclose(quat_rotate_inv(q, quat_rotate(q, v)), v, atol=1e-12)
        assert np.allclose(r @ r.T, np.eye(3), atol=1e-12)
        assert np.linalg.det(r) == pytest.approx(1.0)


def test_dcm_roundtrip():
    for _ in range(100):
        q = rand_quat()
        q2 = dcm_to_quat(quat_to_dcm(q))
        assert abs(np.dot(q, q2)) == pytest.approx(1.0, abs=1e-12)


def test_composition_order():
    qa, qb = rand_quat(), rand_quat()
    v = (0.3, -1.2, 2.0)
    assert np.allclose(quat_rotate(quat_mul(qa, qb), v), quat_rotate(qa, quat_rotate(qb, v)), atol=1e-12)


def test_axis_angle_90deg_about_z():
    q = quat_from_axis_angle((0, 0, 1), math.pi / 2)
    assert np.allclose(quat_rotate(q, (1, 0, 0)), (0, 1, 0), atol=1e-12)


@pytest.mark.parametrize("el,az", [(90, 0), (60, 45), (30, 200), (10, 300), (0, 0), (45, 90)])
def test_pointing_places_nose_axis(el, az):
    q = quat_from_pointing(math.radians(el), math.radians(az))
    nose = quat_rotate(q, (1, 0, 0))
    e, a = math.radians(el), math.radians(az)
    expect = (math.cos(e) * math.sin(a), math.cos(e) * math.cos(a), math.sin(e))  # E, N, U
    assert np.allclose(nose, expect, atol=1e-12)


def test_level_north_is_frd():
    """Level, heading north: body y -> East, body z -> Down (aircraft FRD)."""
    q = quat_from_pointing(0.0, 0.0)
    assert np.allclose(quat_rotate(q, (1, 0, 0)), (0, 1, 0), atol=1e-12)  # nose = North
    assert np.allclose(quat_rotate(q, (0, 1, 0)), (1, 0, 0), atol=1e-12)  # right = East
    assert np.allclose(quat_rotate(q, (0, 0, 1)), (0, 0, -1), atol=1e-12)  # down


def test_euler_extraction():
    q = quat_from_pointing(math.radians(30), math.radians(120), math.radians(25))
    yaw, pitch, roll = quat_to_euler(q)
    assert math.degrees(yaw) == pytest.approx(120.0, abs=1e-9)
    assert math.degrees(pitch) == pytest.approx(30.0, abs=1e-9)
    assert math.degrees(roll) == pytest.approx(25.0, abs=1e-9)


def test_error_vector_small_rotation():
    q = rand_quat()
    dq = quat_from_axis_angle((0.2, -0.5, 0.8), 0.01)
    q_target = quat_mul(q, dq)  # target is "q rotated by dq in body frame"
    e = quat_error_vector(q_target, q)
    axis = np.array((0.2, -0.5, 0.8))
    axis /= np.linalg.norm(axis)
    assert np.allclose(e, axis * 0.01, atol=1e-6)
    assert quat_conj(q)[0] == q[0]


def test_angle_between():
    assert angle_between((1, 0, 0), (0, 1, 0)) == pytest.approx(math.pi / 2)
    assert angle_between((1, 0, 0), (-1, 0, 0)) == pytest.approx(math.pi)
