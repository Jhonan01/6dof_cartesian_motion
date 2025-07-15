import unittest
from kinematics import forward_kin, get_angles
from trajectory import inside_of_limits, lerp_pose
from math import radians

class TestKinematics(unittest.TestCase):

    def test_forward_kinematics_output(self):
        X, Y, Z = forward_kin(0, 0, 0, 0, 0, 0)
        self.assertEqual(len(X), 7)
        self.assertEqual(len(Y), 7)
        self.assertEqual(len(Z), 7)

    def test_inverse_kinematics_valid(self):
        x, y, z = 1.0, 1.0, 1.0
        roll, pitch, yaw = 0, 0, 0
        q = get_angles(x, y, z, roll, pitch, yaw)
        self.assertEqual(len(q), 6)
        self.assertTrue(inside_of_limits(q))

    def test_out_of_limits(self):
        # It forces a value out of the limits of joint 2 (expected False)
        angles = (0, radians(100), 0, 0, 0, 0)
        self.assertFalse(inside_of_limits(angles))

    def test_lerp_pose(self):
        start = [0, 0, 0, 0, 0, 0]
        end = [1, 2, 3, 0.1, 0.2, 0.3]
        steps = 10

        traj = lerp_pose(start, end, steps)

        # It must have steps+1 poses
        self.assertEqual(len(traj), steps + 1)

        # Each pose must have 6 values
        for pose in traj:
            self.assertEqual(len(pose), 6)

        # The first pose must be equal to the start
        self.assertEqual(traj[0], start)

        # The last pose must be equal to the end
        self.assertEqual(traj[-1], end)

if __name__ == '__main__':
    unittest.main()
