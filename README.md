# organizing_obj_task_motion_planner
Task and motion planner for organising objects in ros environment. The robot consists of a KUKA LWR arm and a barrett hand. 

This package has depedencies on kuka_fri_bridge, bhand_controller. 

1. Just run to start:

roslaunch organizing_obj_task_motion_planner organizing_obj_simulation.launch

2. publish a target pose of the objects to run the planner, for example:

rostopic pub -1 /OBJECT/target_poses organizing_obj_task_motion_planner/ObjectPoses '{poses: [-0.00924, -0.1255, 82.33719603, -0.20073, -0.08141, 144.13363736, -0.04941, 0.11, 39.73772348, -0.2369, 0.07923, 23]}'

3. robot begins to organise the table.
