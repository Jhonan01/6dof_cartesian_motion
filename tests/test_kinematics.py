import unittest
from kinematics import forward_kin, get_angles
from trajectory import inside_of_limits
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

if __name__ == '__main__':
    unittest.main()
