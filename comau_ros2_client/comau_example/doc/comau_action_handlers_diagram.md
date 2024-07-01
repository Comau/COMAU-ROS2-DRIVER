<div hidden>

  ```plantuml
  @startuml comau_action_handlers_component_diagram
  skinparam componentStyle uml2
  skinparam defaultTextAlignment center

  left to right direction

  package "ROS Action messages" as AM{
    () "ExecuteJointTrajectory" as execj
    () "ExecuteCartesianTrajectory" as execc
    () "MoveToJointsMoveIt " as movej
    () "MoveToPoseMoveIt " as movec
  }

  package "ROS Action Servers " {
    execj -down- [execute_joint_trajectory_handler] 
    execc -down- [execute_cartesian_trajectory_handler] 
    movej -down- [move_joints_handler]
    movec -down- [move_topose_handler]

    [move_joints_handler] -> execj
    [move_topose_handler] -> execj

  } 


  package "Comau C5G controller"{
      [state_server (NOHOLD) ]
      [motion_server (NOHOLD)] -left- [motion_handler (HOLD) ]
  }


  node "TCP/IP comm"{
  [state_server (NOHOLD) ] <-up->  StateClient
  [motion_server (NOHOLD)] <-up-> MotionClient

  [execute_joint_trajectory_handler] <-down- StateClient
  [execute_joint_trajectory_handler] -down-> MotionClient
  [execute_cartesian_trajectory_handler] <-down- StateClient
  [execute_cartesian_trajectory_handler] -down-> MotionClient
  }
  




  ' note left of AM : See comau_msgs package

  @enduml
  ```
  
</div>