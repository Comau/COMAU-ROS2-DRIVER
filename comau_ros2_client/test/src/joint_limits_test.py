import os
import unittest
from ament_index_python.packages import get_package_share_directory
from urdf_parser_py.urdf import URDF

class YourTestClass(unittest.TestCase):

  @classmethod
  def setUpClass(cls):               
      cls.robot_data = [
          {
            "robot_name": "aura",
            "urdf_file_path": os.path.join(get_package_share_directory('aura_description'), 'urdf', 'aura', 'aura_robot.urdf'),
            "expected_joint_count": 9,
            "expected_joints": [
                "joint_1",
                "joint_2",
                "joint_3" ,          
                "joint_3m",
                "joint_4",
                "joint_5",
                "joint_6",
                "fixed_ee_joint",
                "world_joint"
             ],            
             "expected_joint_limits": {
              # IF use_mimic  False
                "joint_1": {'lower': -3.14,   'upper': 3.14,     'effort': 6300.0, 'velocity': 2.0},
                "joint_2": {'lower': -0.34,   'upper': 1.48,     'effort': 3300.0, 'velocity': 2.0},   
                "joint_3": {'lower': -2.2607, 'upper': -0.69079, 'effort': 1000.0, 'velocity': 2.0},   
                "joint_4": {'lower': -3.14,   'upper': 3.14,     'effort': 60.0,   'velocity': 2.0},
                "joint_5": {'lower': -3.14,   'upper': 3.14,     'effort': 60.0,   'velocity': 2.0},
                "joint_6": {'lower': -3.49,   'upper': 3.49,     'effort': 60.0,   'velocity': 2.0},
              },
              # IF use_mimic  True             
              #   "joint_1":  {'lower': -3.14, 'upper': 3.14, 'effort': 6300.0, 'velocity': 2.0},
              #   "joint_2":  {'lower': -0.34, 'upper': 1.48, 'effort': 3300.0, 'velocity': 2.0},              
              #   "joint_3m": {'lower': -3.8397, 'upper': -0.8726, 'effort': 1000.0, 'velocity': 2.0}, 
              #   "joint_4":  {'lower': -3.14, 'upper': 3.14, 'effort': 60.0, 'velocity': 2.0},
              #   "joint_5":  {'lower': -3.14, 'upper': 3.14, 'effort': 60.0, 'velocity': 2.0},
              #   "joint_6":  {'lower': -3.49, 'upper': 3.49, 'effort': 60.0, 'velocity': 2.0},
              # },
          },
          
          {       
            "robot_name": "nj4-110",
            "urdf_file_path": os.path.join(get_package_share_directory('nj4-110_description'), 'urdf', 'nj4-110', 'nj4-110_robot.urdf'),
            "expected_joint_count": 8,
            "expected_joints": [
              "joint_1",
              "joint_2",
              "joint_3",
              "joint_4",
              "joint_5",
              "joint_6",
              "fixed_ee_joint",
              "world_joint"              
            ],
            "expected_joint_limits": {             
               "joint_1": {'lower': -3.142, 'upper': 3.142, 'effort': 100000.00,  'velocity': 20.0},
               "joint_2": {'lower': -2.182, 'upper': 1.047, 'effort': 100000.00,  'velocity': 20.0},   
               "joint_3": {'lower': -1.348, 'upper': 1.222, 'effort': 1000000.0,  'velocity': 20.0},   
               "joint_4": {'lower': -3.49,  'upper': 3.49,  'effort': 1000000.00, 'velocity': 20.0},
               "joint_5": {'lower': -3.49,  'upper': 3.49,  'effort': 100000.00,  'velocity': 20.0},
               "joint_6": {'lower': -3.49,  'upper': 3.49,  'effort': 1000000.00, 'velocity': 20.0},
            }
          },

          {       
            "robot_name": "nj4-170-29",
            "urdf_file_path": os.path.join(get_package_share_directory('nj4-170-29_description'), 'urdf', 'nj4-170-29', 'nj4-170-29_robot.urdf'),
            "expected_joint_count": 14,
            "expected_joints": [
              "joint_1",
              "joint_2",
              "joint_2m",
              "joint_3",
              "joint_4",
              "joint_5",
              "joint_6",
              "fixed_ee_joint",
              "world_joint", 
            # joints: parallel bar joints
              "joint_2_1",
              "joint_2_2"  , 
              "joint_2_2_5" ,  
              "joint_3_1"  ,  
              "joint_3_1_5" ,  
            ],
            "expected_joint_limits": {             
              "joint_1":  {'lower': -3.1416,  'upper': 3.1416,   'effort': 100, 'velocity': 1.75},
              "joint_2":  {'lower': -0.34907, 'upper': 1.4835,   'effort': 70,  'velocity': 1.48},   
              "joint_2m": {'lower': -3.8397,  'upper': -0.87267, 'effort': 70,  'velocity': 1.48},   
              "joint_3":  {'lower': -3.8397,  'upper': -0.87267, 'effort': 100, 'velocity': 1.75},
              "joint_4":  {'lower': -3.1416,  'upper': 3.1416,   'effort': 100, 'velocity': 2.27},
              "joint_5":  {'lower': -3.1416,  'upper': 3.1416,   'effort': 100, 'velocity': 2.44},
              "joint_6":  {'lower': -3.1416,  'upper': 3.1416,   'effort': 70,  'velocity': 3.32},
            }
          },

          {       
             "robot_name": "nj220",
             "urdf_file_path": os.path.join(get_package_share_directory('nj220_description'), 'urdf', 'nj220', 'nj220_robot.urdf'),
             "expected_joint_count": 8,
             "expected_joints": [
               "joint_1",
               "joint_2",
               "joint_3",
               "joint_4",
               "joint_5",
               "joint_6",
               "joint_ee_link",
               "world_joint",  
             ],
             "expected_joint_limits": {             
               "joint_1": {'lower': -3.14,  'upper': 3.14,  'effort': 10000.00, 'velocity': 20.0},
               "joint_2": {'lower': -1.309, 'upper': 1.658, 'effort': 10000.00, 'velocity': 20.0},   
               "joint_3": {'lower': -1.396, 'upper': 1.238, 'effort': 10000.00, 'velocity': 20.0},   
               "joint_4": {'lower': -3.14,  'upper': 3.14, 'effort': 10000.00, 'velocity': 20.0},
               "joint_5": {'lower': -2.182, 'upper': 2.182, 'effort': 10000.00, 'velocity': 20.0},
               "joint_6": {'lower': -3.14,  'upper': 3.14,  'effort': 10000.00, 'velocity': 20.0},
             }
          },           

          {       
             "robot_name": "racer5-0-80",
             "urdf_file_path": os.path.join(get_package_share_directory('racer5-0-80_description'), 'urdf', 'racer5-0-80', 'racer5-0-80_robot.urdf'),
             "expected_joint_count": 8,
             "expected_joints": [
               "joint_1",
               "joint_2",
               "joint_3",
               "joint_4",
               "joint_5",
               "joint_6",
               "joint_ee_link",
               "world_joint", 
             ],
             "expected_joint_limits": {             
               "joint_1": {'lower': -2.966, 'upper': 2.966, 'effort': 100000.00, 'velocity': 20.00},
               "joint_2": {'lower': -1.99,  'upper': 1.116, 'effort': 100000.00, 'velocity': 20.00},    
               "joint_3": {'lower': -2.62,  'upper': 1.483, 'effort': 100000.00, 'velocity': 20.00},
               "joint_4": {'lower': -3.49,  'upper': 3.49,  'effort': 100000.00, 'velocity': 20.00},
               "joint_5": {'lower': -1.83,  'upper': 1.83,  'effort': 100000.00, 'velocity': 20.00},
               "joint_6": {'lower': -47.10, 'upper': 47.10, 'effort': 100000.00, 'velocity': 20.00},
             }
          },  

          {       
             "robot_name": "racer5-cobot",
             "urdf_file_path": os.path.join(get_package_share_directory('racer5-cobot_description'), 'urdf', 'racer5-cobot', 'racer5-cobot_robot.urdf'),
             "expected_joint_count": 8,
             "expected_joints": [
               "joint_1",
               "joint_2",
               "joint_3",
               "joint_4",
               "joint_5",
               "joint_6",
               "joint_ee_link",
               "world_joint", 
             ],
             "expected_joint_limits": {             
                "joint_1": {'lower': -2.966, 'upper': 2.966, 'effort': 100000.00, 'velocity': 20.00},
                "joint_2": {'lower': -1.116, 'upper': 1.99,  'effort': 100000.00, 'velocity': 20.00},    
                "joint_3": {'lower': -2.62,  'upper': 1.483, 'effort': 100000.00, 'velocity': 20.00},
                "joint_4": {'lower': -3.49,  'upper': 3.49,  'effort': 100000.00, 'velocity': 20.00},
                "joint_5": {'lower': -1.83,  'upper': 1.83,  'effort': 100000.00, 'velocity': 20.00},
                "joint_6": {'lower': -100,   'upper': 100,   'effort': 100000.00, 'velocity': 20.00},
             }
          },           

          {       
             "robot_name": "racer5-cobot-rail",
             "urdf_file_path": os.path.join(get_package_share_directory('racer5-cobot-rail_description'), 'urdf', 'racer5-cobot', 'racer5-cobot-rail_robot.urdf'),
             "expected_joint_count": 10,
             "expected_joints": [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
                "joint_7",
                "rail_joint",
                "joint_ee_link",
                "world_joint", 
             ],
             "expected_joint_limits": {             
                "joint_1": {'lower': -2.966, 'upper': 2.966, 'effort': 100000.00, 'velocity': 20.00},
                "joint_2": {'lower': -1.116, 'upper': 1.99,  'effort': 100000.00, 'velocity': 20.00},    
                "joint_3": {'lower': -2.62,  'upper': 1.483, 'effort': 100000.00, 'velocity': 20.00},
                "joint_4": {'lower': -3.49,  'upper': 3.49,  'effort': 100000.00, 'velocity': 20.00},
                "joint_5": {'lower': -1.83,  'upper': 1.83,  'effort': 100000.00, 'velocity': 20.00},
                "joint_6": {'lower': -3.14,  'upper': 3.14,  'effort': 100000.00, 'velocity': 20.00},
                "joint_7": {'lower': -0.0,   'upper': 2.0,   'effort': 100000.00, 'velocity': 20.00},             
             }                
          }, 

          {       
             "robot_name": "racer7-14",
             "urdf_file_path": os.path.join(get_package_share_directory('racer7-14_description'), 'urdf', 'racer7-14', 'racer7-14_robot.urdf'),
             "expected_joint_count": 8,
             "expected_joints": [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
                "joint_ee_link",
                "world_joint", 
             ],
             "expected_joint_limits": {             
                "joint_1": {'lower': -2.8797932658, 'upper': 2.8797932658, 'effort': 10000.00, 'velocity': 20.00},
                "joint_2": {'lower': -1.4835298642, 'upper': 2.7052603406, 'effort': 10000.00, 'velocity': 20.00},    
                "joint_3": {'lower': -2.9321531434, 'upper': 0.0,          'effort': 10000.00, 'velocity': 20.00},
                "joint_4": {'lower': -3.6651914292, 'upper': 3.6651914292, 'effort': 10000.00, 'velocity': 20.00},
                "joint_5": {'lower': -2.3561944902, 'upper': 2.3561944902, 'effort': 10000.00, 'velocity': 20.00},
                "joint_6": {'lower': -47.1238898,   'upper': 47.1238898,   'effort': 10000.00, 'velocity': 20.00},
             }                
          }, 
      ]
      
      # Inizializzazione liste
      cls.passed_robots_joint_number = []
      cls.passed_robots_joint_limits = []

      cls.not_passed_robots_joint_number = []
      cls.not_passed_robots_joint_limits = []

  def test_joint_number(self):
    for robot_info in self.robot_data:
        with self.subTest(robot_info["robot_name"]):
          all_tests_passed = True  # Variabile di controllo    
          robot = URDF.from_xml_file(robot_info["urdf_file_path"])
          found_joints = [joint.name for joint in robot.joints]
  
          # Verifica se il numero di giunti trovati corrisponde al numero atteso
          try:
              self.assertEqual(len(found_joints), robot_info["expected_joint_count"],
                               f"Il NUMERO di giunti trovati nel file URDF per il robot {robot_info['robot_name']} non corrisponde al numero atteso di giunti.")
          except AssertionError as error:
              print(f"  *****     TEST FALLITO per il robot {robot_info['robot_name']}: {error}    *****")
              all_tests_passed = False 

          # Verifica che ciascun giunto atteso sia presente nei giunti trovati
          for joint in robot_info["expected_joints"]:
              with self.subTest(joint=joint):
                  if joint not in found_joints:
                      print(f"Il GIUNTO {joint} ATTESO NON è stato TROVATO nel file URDF del robot {robot_info['robot_name']}")
                      all_tests_passed = False  # Imposta la variabile di controllo su False se il test fallisce
  
          # Verifica che tutti i giunti nel file URDF siano presenti tra quelli attesi
          for joint in found_joints:
              with self.subTest(joint=joint):
                  if joint not in robot_info["expected_joints"]:
                      print(f"Il GIUNTO {joint} trovato nel file URDF del robot {robot_info['robot_name']} NON è tra quelli ATTESI.")
                      all_tests_passed = False  # Imposta la variabile di controllo su False se il test fallisce
  
          # Se tutti i controlli passano senza errori, aggiungi il nome del robot alla lista
          if all_tests_passed:
              self.passed_robots_joint_number.append(robot_info["robot_name"])
          else:
              self.not_passed_robots_joint_number.append(robot_info["robot_name"])

  def test_joint_limits(self):
    for robot_info in self.robot_data:
        all_joints_passed = True
        with self.subTest(robot_info["robot_name"]):
          robot = URDF.from_xml_file(robot_info["urdf_file_path"])
          for joint in robot.joints:
              if joint.name in robot_info["expected_joint_limits"]:
                  expected = robot_info["expected_joint_limits"][joint.name]
                  actual = joint.limit
                  try:
                    self.assertAlmostEqual(actual.lower, expected['lower'], places=3, msg=f"Lower limit mismatch for the robot {robot_info['robot_name']} - joint {joint.name}")
                    self.assertAlmostEqual(actual.upper, expected['upper'], places=3, msg=f"Upper limit mismatch for the robot {robot_info['robot_name']} - joint {joint.name}")
                    self.assertAlmostEqual(actual.effort, expected['effort'], places=3, msg=f"Effort limit mismatch for the robot {robot_info['robot_name']} - joint {joint.name}")
                    self.assertAlmostEqual(actual.velocity, expected['velocity'], places=3, msg=f"Velocity limit mismatch for the robot {robot_info['robot_name']} - joint {joint.name}")
                  
                    #self.passed_robots_joint_limits.append(robot_info["robot_name"])
                  
                  except AssertionError as error:
                    print(f" *****     TEST FALLITO  per il robot {robot_info['robot_name']} - joint {joint.name}: {error}")
                    all_joints_passed = False
                    self.not_passed_robots_joint_limits.append(robot_info["robot_name"])
                  
          if all_joints_passed:
            self.passed_robots_joint_limits.append(robot_info["robot_name"])

  @classmethod
  def tearDownClass(cls):
      print("Test sul numero di giunti completati per i seguenti robot:")
      for robot_name in cls.passed_robots_joint_number:
          print(f"- {robot_name}")
  
      print("Test sui limiti di giunto completati per i seguenti robot:")
      for robot_name in cls.passed_robots_joint_limits:
          print(f"- {robot_name}")  
          
      print("Test fallito sul numero di giunto per i seguenti robot:")
      for robot_name in cls.not_passed_robots_joint_number:
          print(f"- {robot_name}")
  
      print("Test fallito sui limiti di giunto per i seguenti robot:")
      for robot_name in cls.not_passed_robots_joint_limits:
          print(f"- {robot_name}")              

if __name__ == '__main__':
  unittest.main()