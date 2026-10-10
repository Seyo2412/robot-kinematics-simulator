import numpy as np
import pytest
from src.kinematics import forward_kinematics_3r, inverse_kinematics_3r, RobotArm6DOF

def test_fk_zero_configuration():

    lengths = [2.0, 1.5, 1.0]
    angles = [0.0, 0.0, 0.0]
    pts = forward_kinematics_3r(angles, lengths)

    expected_x = sum(lengths)
    assert np.isclose(pts[-1, 0], expected_x), "Arm sollte flach bei x=4.5 liegen"
    assert np.isclose(pts[-1, 1], 0.0), "Y-Koordinate muss 0 sein"

def test_ik_reaches_target():

    lengths = [2.0, 1.5, 1.0]
    target_x, target_y = 2.0, 2.0
    target_phi = np.radians(30.0)

    calc_angles = inverse_kinematics_3r(target_x, target_y, target_phi, lengths)

    pts = forward_kinematics_3r(calc_angles, lengths)
    reached_x, reached_y = pts[-1, 0], pts[-1, 1]

    assert np.isclose(reached_x, target_x, atol=1e-5)
    assert np.isclose(reached_y, target_y, atol=1e-5)

def test_ik_out_of_reach_raises_error():

    lengths = [2.0, 1.5, 1.0]
    with pytest.raises(ValueError):
        inverse_kinematics_3r(10.0, 10.0, 0.0, lengths)

def test_6dof_base_position():

    robot = RobotArm6DOF()
    pts = robot.forward_kinematics([0]*6)
    assert np.allclose(pts[0], [0.0, 0.0, 0.0])