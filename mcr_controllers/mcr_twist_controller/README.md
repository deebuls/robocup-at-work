## Description
This package controls a robot manipulator, in Cartesian Space,
by publishing a twist to reduce the difference between two poses.

## Usage
1. Launch the component:
```roslaunch mcr_twist_controller twist_controller.launch```
2. Subscribe to the result of the component:
```rostopic echo /arm_1/arm_controller/cartesian_velocity_command```
3. Publish the (component-wise) pose difference (example):
```rostopic pub /twist_controller/pose_error mcr_manipulation_msgs/ComponentWiseCartesianDifference '{header: {frame_id: "/arm_link_5"}, linear: {x: 0.05, y: 0.0, z: 0.02}, angular: {x: 0.0, y: 0.0, z: 0.0 }}'```
4. Start the component:
```rostopic pub /twist_controller/event_in std_msgs/String 'e_start'```

## Usage (with mock-up)
1. Launch the mock-up:
```roslaunch mcr_twist_controller pose_error_mock_up_gui.launch```
2. Launch the component:
```roslaunch mcr_twist_controller twist_controller.launch```
3. Subscribe to the result of the component:
```rostopic echo /arm_1/arm_controller/cartesian_velocity_command```
4. Start the component:
```rostopic pub /twist_controller/event_in std_msgs/String 'e_start'```

### To stop the component:
```rostopic pub /twist_controller/event_in std_msgs/String  'e_stop'```