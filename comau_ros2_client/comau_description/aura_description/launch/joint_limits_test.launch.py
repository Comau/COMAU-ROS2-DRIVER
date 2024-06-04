import os
import launch
import launch_ros
import launch_testing
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import pytest
import unittest
from ament_index_python.packages import get_package_share_directory
from urdf_parser_py.urdf import URDF

@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        ReadyToTest()
    ])

class TestJointLimits(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Percorso al file URDF
        urdf_file = os.path.join(get_package_share_directory('aura_description'), 'robots', 'aura_robot.urdf')

        # Carica il file URDF
        cls.robot = URDF.from_xml_file(urdf_file)

    def test_joint_limits(self):
        # Definisci i limiti attesi per i giunti
        expected_limits = {
            'joint1': {'lower': -1.57, 'upper': 1.57, 'effort': 100.0, 'velocity': 1.0},
            'joint2': {'lower': -0.785, 'upper': 0.785, 'effort': 50.0, 'velocity': 0.5},
            # Aggiungi altri giunti e i loro limiti attesi qui
        }

        for joint_name, limits in expected_limits.items():
            with self.subTest(joint=joint_name):
                joint = self.robot.joint_map[joint_name]
                self.assertIsNotNone(joint.limit, f"No limits found for joint {joint_name}")

                # Verifica i limiti dei giunti
                self.assertAlmostEqual(joint.limit.lower, limits['lower'], places=2, msg=f"Lower limit mismatch for joint {joint_name}")
                self.assertAlmostEqual(joint.limit.upper, limits['upper'], places=2, msg=f"Upper limit mismatch for joint {joint_name}")
                self.assertAlmostEqual(joint.limit.effort, limits['effort'], places=2, msg=f"Effort limit mismatch for joint {joint_name}")
                self.assertAlmostEqual(joint.limit.velocity, limits['velocity'], places=2, msg=f"Velocity limit mismatch for joint {joint_name}")

# Per eseguire il test con pytest
@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        ReadyToTest(),
    ])

if __name__ == '__main__':
    import ros2launch
    ros2launch.main(