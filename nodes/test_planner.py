from organizing_obj_task_motion_planner_py import *

crrtPoses = [-0.25456, -0.14179, 0, -0.11312, 0.06053, 176.49369886, 0.00579, -0.09805, 136.21972607, -0.26054, 0.13388, 20]
targetPoses = [-0.00924, -0.1255, 82.33719603, -0.20073, -0.08141, 144.13363736, -0.04941, 0.11, 39.73772348, -0.2369, 0.07923, 23]

objectNames = ['domino_sugar', 'red_metal_bowl', 'red_metal_cup', 'master_chef_coffee']
jointStateCrrt = [0,0,0,0,0,0,0]

jointValueSeq = giveActionSeq(objectNames, crrtPoses, targetPoses, jointStateCrrt)

