<div hidden>

  ```plantuml
  @startuml comau_driver_component_diagram
  skinparam componentStyle uml2
  skinparam defaultTextAlignment center

  left to right direction

  package "MoveIt Handlers" as moveit <<Folder>> {
    class  "MoveJointsHandlerServer" as movetojoints{
      +createExecJointTrajClient()
      -executeCallback()
    }
    class  "MoveToPoseHandlerServer" as movetopose{
      +createExecJointTrajClient()
      -executeCallback()
    }
  }

  package "Comau Handlers" as cmHandlers <<Folder>> {
    class  "ExecuteJointTrajectoryHandler" as execj{
      -executeCallback()
    }
    class  "ExecuteCartesianTrajectoryHandler" as execc{
      -executeCallback()
    }
  }

  package "Comau Hardware Interface" as cmHW <<Folder>> {
    abstract class  "ComauHWBase" as base{
        +virtual read()
        +update()
        +virtual write()
    }
    interface  "ComauHWInterface" as interface{
        +read()
        +write()
    }
    class      "ComauHWControlLoop" as cloop{
        +run()
        -update()
    }
  }

  package "Comau Driver" as cmDriver <<Folder>> {
    class "ComauRobot" as crobot{
    +writeJointCommand()
    +getJointPosition()
    +getEePosition()
    +getTimeStamp()
    +getStatus()
    }
  }

  package "Comau TCP Interface" as cmTCP <<Folder>> {
    class "ComauClientBase" as tcp_base
    class "StateClient" as tcp_state
    class "MotionClient" as tcp_motion
  }

  package "Comau C5G Controller" as C5G <<Folder>> {
    class "state_server" as state_server
    class "motion_server" as motion_server
    class "motion_handler(HOLD)" as motion_handler
  }

  ''' Sensor Tracking
  package "Sensor Tracking" as SnsTrk <<Folder>> {
    class "SensorDataSource" as sensor_converter{
      +SensorDataConverter()
      +TwistStamped
    }
  }
  ''''

  ' Connections between classes

  ' Inside comau hardware
  base "inherits from" +--> interface
  interface -up-> cloop
  ' Inside TCP
  tcp_base +--> tcp_state
  tcp_base +--> tcp_motion
  ' Moveit to hardware
  movetojoints *--> interface
  movetopose *--> interface
  ' Moveit to execute server
  movetojoints *--> execj
  movetopose *--> execj
  ' Execute server to Robot Driver
  execj *--> crobot
  execc *--> crobot
  ' Hardware Interface to Robot Driver
  interface --> crobot
  ' Robot Driver TO TCP
  crobot *--> tcp_motion
  crobot *--> tcp_state
  ' C5G TO tcp
  state_server <--* tcp_state
  motion_server <--* tcp_motion
  motion_server --> motion_handler
  ' Inside Sensor tracking
  'sensor_data *--> sensor_converter
  ' Sensor  To Hardware
  sensor_converter *--> interface
  ' Notes
  note as N1
    <img:LMS_Logo.png>
  end note

  @enduml
  ```
  
</div>