from openravepy import *
import copy
from types import *
from datetime import datetime
import IPython
import random
import math, time
import numpy as np
import sys
import pdb
# pdb.set_trace() #breakpoint
sysPathThisFile = sys.path[0]
plannerPath = sys.path[1]
sys.path.insert(-1, sysPathThisFile + '/..')
sys.path.insert(-1, sysPathThisFile + '/Yues_class_definition')
sys.path.insert(-1, sysPathThisFile + '/imported_library')
sys.path.insert(-1, sysPathThisFile + '/../../robots/Experiment_LASA/Experiment_LASA/imported_library')

# pdb.set_trace() #breakpoint
from Dijkstra_shortest_path import *
from Yues_GUI_module import *
from openravepy import misc
from scipy import linalg
# if not __openravepy_build_doc__:
from openravepy import *
from numpy import * 
import time, threading
from collections import Counter
from numpy.linalg import inv
from scipy.optimize import *
from math import radians
# import rospy
from copy import copy, deepcopy
from GraspFun import *
from Yues_functions import * 

# env.Load('/home/yue/catkin_ws/src/organizing_obj_task_motion_planner/nodes/OpenRaveSim/robots/Experiment_LASA/Scene_Experiment_LASA.env.xml')

# sys.path.insert(0, '/home/yue/openrave/python/examples')
from simplegrasping import *

def giveActionSeq(objectNames, crrtPoses, targetPoses, jointStateCrrt):
	print 'running planner...'
	#############################################################################################################
	# set up environment
	#############################################################################################################
	env = Environment() # create the environment
	env.SetDebugLevel(DebugLevel.Fatal)
	env.SetViewer('qtcoin') # start the viewer
	envPath = plannerPath + '/OpenRaveSim/robots/Experiment_LASA/Scene_Experiment_LASA.env.xml'
	env.Load(envPath) # load a scene

	# pdb.set_trace() #breakpoint

	tableEnum1 = 2
	tableEnum2 = 3
	table1 = env.GetBodies()[tableEnum1]
	table2 = env.GetBodies()[tableEnum2]

	# table 1 area
	area = np.zeros([5,1])
	area[0] = env.GetBodies()[tableEnum1].GetTransform()[0,3] # X position of table
	area[1] = env.GetBodies()[tableEnum1].GetTransform()[1,3] # Y position of table
	# area[2] = 0.8 # orginial length of table in X
	# area[3] = 0.8 # orginial  length of table in Y
	# area[4] = 0.738 # orginial height in Z
	area[2] = 0.5 # new length of table in X
	area[3] = 0.5 # new length of table in Y
	area[4] = env.GetBodies()[tableEnum1].GetTransform()[2,3] + 0.055 # new height in Z
	# pdb.set_trace() #breakpoint
	# add objects --------------------------------------------------------------------------------------------
	objectList = [] # [0]. obj class, [1]. obj name, [2]. obj image, [3]. Z-rotation sample?
	enumObjBegin = 3
	enumobj = 0
	onlyPushObjEnum = []
	initialTemporaryTransform = np.eye(4)
	initialTemporaryTransform[0,3] = 0.5


	"""
	# plate
	env.Load('../../robots/Kitchen_LASA/Kitchen_plate2.kinbody.xml')
	enumobj = enumobj + 1 
	enumPlate = enumobj
	objectList.append(env.GetBodies()[enumObjBegin+enumPlate])
	env.GetBodies()[enumObjBegin+enumPlate].SetTransform(initialTemporaryTransform)
	"""
	"""
	# knife
	env.Load('../../robots/Kitchen_LASA/Kitchen_knife.kinbody.xml')
	enumobj = enumobj + 1 
	enumKnife = enumobj
	objectList.append(env.GetBodies()[enumObjBegin+enumKnife])
	env.GetBodies()[enumObjBegin+enumKnife].SetTransform(initialTemporaryTransform)
	"""
	"""
	# fork
	objPath = plannerPath + '/OpenRaveSim/robots/Kitchen_LASA/Kitchen_fork.kinbody.xml'
	env.Load(objPath)
	enumobj = enumobj + 1 
	enumFork = enumobj
	objectList.append(env.GetBodies()[enumObjBegin+enumFork])
	env.GetBodies()[enumObjBegin+enumFork].SetTransform(initialTemporaryTransform)
	

	# domino_sugar
	objPath = plannerPath + '/OpenRaveSim/robots/Kitchen_LASA/domino_sugar.kinbody.xml'
	env.Load(objPath)
	enumobj = enumobj + 1 
	enumDominoSugar = enumobj
	objectList.append(env.GetBodies()[enumObjBegin+enumDominoSugar])
	env.GetBodies()[enumObjBegin+enumDominoSugar].SetTransform(initialTemporaryTransform)
	"""



	# master_chef_coffee
	objPath = plannerPath + '/OpenRaveSim/robots/Kitchen_LASA/master_chef_coffee.kinbody.xml'
	env.Load(objPath)
	enumobj = enumobj + 1 
	enumMasterChefCoffee = enumobj
	objectList.append(env.GetBodies()[enumObjBegin+enumMasterChefCoffee])
	env.GetBodies()[enumObjBegin+enumMasterChefCoffee].SetTransform(initialTemporaryTransform)


	# red_metal_cup
	objPath = plannerPath + '/OpenRaveSim/robots/Kitchen_LASA/red_metal_cup.kinbody.xml'
	env.Load(objPath)
	enumobj = enumobj + 1 
	enumRedMetalCup = enumobj
	objectList.append(env.GetBodies()[enumObjBegin+enumRedMetalCup])
	env.GetBodies()[enumObjBegin+enumRedMetalCup].SetTransform(initialTemporaryTransform)

	# red_metal_bowl
	objPath = plannerPath + '/OpenRaveSim/robots/Kitchen_LASA/red_metal_bowl.kinbody.xml'
	env.Load(objPath)
	enumobj = enumobj + 1 
	enumRedMetalBowl = enumobj
	objectList.append(env.GetBodies()[enumObjBegin+enumRedMetalBowl])
	env.GetBodies()[enumObjBegin+enumRedMetalBowl].SetTransform(initialTemporaryTransform)

	
	"""
	# glass
	objPath = plannerPath + '/OpenRaveSim/robots/Kitchen_LASA/Kitchen_glass.kinbody.xml'
	env.Load(objPath)
	enumobj = enumobj + 1 
	enumGlass = enumobj
	objectList.append(env.GetBodies()[enumObjBegin+enumGlass])
	env.GetBodies()[enumObjBegin+enumGlass].SetTransform(initialTemporaryTransform)
	# pdb.set_trace() #breakpoint

	# cup
	objPath = plannerPath + '/OpenRaveSim/robots/Kitchen_LASA/Kitchen_cup.kinbody.xml'
	env.Load(objPath)
	enumobj = enumobj + 1 
	enumCup = enumobj
	objectList.append(env.GetBodies()[enumObjBegin+enumCup])
	env.GetBodies()[enumObjBegin+enumCup].SetTransform(initialTemporaryTransform)

	# mug
	objPath = plannerPath + '/OpenRaveSim/robots/Kitchen_LASA/Kitchen_mug.kinbody.xml'
	env.Load(objPath)
	enumobj = enumobj + 1 
	enumMug = enumobj
	objectList.append(env.GetBodies()[enumObjBegin+enumMug])
	env.GetBodies()[enumObjBegin+enumMug].SetTransform(initialTemporaryTransform)
	"""

	"""
	# monitor
	env.Load('../../robots/Kitchen_LASA/Kitchen_monitor.kinbody.xml')
	enumobj = enumobj + 1
	enumMonitor = enumobj
	objectList.append(env.GetBodies()[enumObjBegin+enumMonitor])
	env.GetBodies()[enumObjBegin+enumMonitor].SetTransform(initialTemporaryTransform)
	onlyPushObjEnum.append(enumMonitor)
	"""
	# pdb.set_trace() #breakpoint
	# set robot manipulator --------------------------------------------------------------------------------------------
	robot1 = env.GetRobots()[0]
	manip1 = robot1.SetActiveManipulator("lwr")
	# pdb.set_trace() #breakpoint
	robot1.SetDOFValues(jointStateCrrt, manip1.GetArmIndices())

	manip_tool1 = manip1.GetEndEffector()
	tool_Transform = manip_tool1.GetTransform()
	
	h4,h5,h6 = PlotFrame(env, tool_Transform, 0.1)

	ikmodel1 = databases.inversekinematics.InverseKinematicsModel(robot = robot1, iktype = IkParameterization.Type.Transform6D)
	if not ikmodel1.load():
	    ikmodel1.autogenerate()
	# transformGoToArm = env.GetBodies()[4].GetTransform() # glass
	# transformGoToArm = [[ -6.76982064e-02,   1.70535383e-23,   9.97705845e-01, -1.17274895e-02],[  8.06436262e-22,   1.00000000e+00,   3.76270725e-23, -5.91917820e-02],[ -9.97705845e-01,   8.07133458e-22,  -6.76982064e-02, 1.15282023e+00],[  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,1.00000000e+00]]
	# handle = []
	# handle.append(PlotFrame(env, transformGoToArm, 0.03))
	# sol_DOF = manip1.FindIKSolution(transformGoToArm, IkFilterOptions.CheckEnvCollisions) # IgnoreJointLimits, CheckEnvCollisions
	# robot1.SetDOFValues(sol_DOF, manip1.GetArmIndices())


	#############################################################################################################################
	# Generate random initial/goal object placement 
	#############################################################################################################################

	# pdb.set_trace() #breakpoint
	initCfgObjList = []
	for i_objectOpenrave in range(shape(objectList)[0]):
		for i_objectInput in range(shape(objectNames)[0]):
			if objectList[i_objectOpenrave].GetName() == objectNames[i_objectInput]:
				posInX = crrtPoses[i_objectInput*3]
				posInY = crrtPoses[i_objectInput*3+1]
				posInZ = area[4]
				zRotDegree = crrtPoses[i_objectInput*3+2]
				initCfgObjList.append([posInX, posInY, zRotDegree])
				zRotRadian = zRotDegree/180*pi
				position = numpy.zeros([4,4])
				position[0:3,3] = [posInX, posInY, posInZ]
				objectTransformNew = giveRotationMatrix3D_4X4('z',zRotRadian) + position
				objectList[i_objectOpenrave].SetTransform(objectTransformNew)
				break
			else:
				continue


	# random goal configuration------------------------------------------------------------------
	newArea = np.zeros([5,1])
	newArea[0] = env.GetBodies()[tableEnum2].GetTransform()[0,3]
	newArea[1] = env.GetBodies()[tableEnum2].GetTransform()[1,3]
	newArea[2] = 0.5
	newArea[3] = 0.5
	newArea[4] = area[4]
	deltaTableX = area[0] - newArea[0]
	deltaTableY = area[1] - newArea[1]
	
	newObjectList = []

	for i_object in range(shape(objectList)[0]):
		newObject = RaveCreateKinBody(env, objectList[i_object].GetXMLId())
		newObject.Clone(objectList[i_object], 0)
		newObjectList.append(newObject)
		env.Add(newObject,True)

	newCfgObjList = []
	# pdb.set_trace() #breakpoint
	for i_objectOpenrave in range(shape(objectList)[0]):
		for i_objectInput in range(shape(objectNames)[0]):
			if objectList[i_objectOpenrave].GetName() == objectNames[i_objectInput]:
				posInX = targetPoses[i_objectInput*3]
				posInY = targetPoses[i_objectInput*3+1]
				posInZ = area[4]
				zRotDegree = targetPoses[i_objectInput*3+2]
				newCfgObjList.append([posInX, posInY, zRotDegree])
				zRotRadian = zRotDegree/180*pi
				position = numpy.zeros([4,4])
				position[0:3,3] = [posInX, posInY - deltaTableY, posInZ]
				objectTransformNew = giveRotationMatrix3D_4X4('z',zRotRadian) + position
				newObjectList[i_objectOpenrave].SetTransform(objectTransformNew)
				break
			else:
				continue

	# newCfgObjList = generateRandomPositionObjectsInARectAreaWithZRot(newArea,newObjectList, env, 'reachable')

	refCfgObjList = newCfgObjList # goal configuration

	"""
	for i_object in range(shape(objectList)[0]):
		refCfgObjList[i_object][0] = refCfgObjList[i_object][0] + deltaTableX
		refCfgObjList[i_object][1] = refCfgObjList[i_object][1] + deltaTableY

	# show reference in original table
	refObjectList = []
	for i_object in range(shape(objectList)[0]):
		refObject = RaveCreateKinBody(env, objectList[i_object].GetXMLId())
		refObject.Clone(objectList[i_object], 0)
		for link in refObject.GetLinks():
				for geom in link.GetGeometries():
					geom.SetTransparency(0.8)
		refTransform = newObjectList[i_object].GetTransform()
		refTransform[0,3] = refTransform[0,3] + deltaTableX
		refTransform[1,3] = refTransform[1,3] + deltaTableY
		refObject.SetTransform(refTransform)
		refObjectList.append(refObject)
		env.Add(refObject,True)
	pdb.set_trace() # breakpoint, make real object untransparent
	"""

	# make real object untransparent
	for i_object in range(shape(objectList)[0]):
		for link in newObjectList[i_object].GetLinks():
				for geom in link.GetGeometries():
					geom.SetTransparency(0)

	# pdb.set_trace() # breakpoint, make real object untransparent

	# pdb.set_trace() #breakpoint	
	#seqActions = [[1,2],[2,2]]
	#seqActHist = [[[1,2],[2,2]],[[2,2],[3,2]],[[1,1],[5,4]]]
	#print checkIfInHistory(seqActions,seqActHist)

	# pdb.set_trace() #breakpoint	
	#############################################################################################################################
	# Rearrange_PRM algorithm
	#############################################################################################################################
	initTransformObjList = giveTransformObjList(objectList) # save the initial configuration for animation
	# reArrangeGraph = InitReArrangeGraph(objectList, initCfgObjList, refCfgObjList)
	transitTransferRRTGraphParameter = np.zeros(100)
	transitTransferRRTGraphParameter[0] = 10000 # numRRTIteration
	transitTransferRRTGraphParameter[1] = 0.3 # collisionBackResidual
	transitTransferRRTGraphParameter[2] = 0.1 # maxCollisionCheckDOFIncrement
	transitTransferRRTGraphParameter[3] = 0.3 # expansionDistance
	transitTransferRRTGraphParameter[4] = 0.3 # goalSampleRate
	transitTransferRRTGraphParameter[5] = 0.5 # goalConnectionRate

	timeCalcBegin = datetime.now()

	reArrangeConnectivity = np.zeros([2, 2])
	reArrangeEdge = []
	reArrangeNode = [initCfgObjList, refCfgObjList]
	reArrangeDistance = giveDistanceReArrangementPRMGraph(reArrangeNode, area)
	reArrangeGraph = [reArrangeNode, reArrangeConnectivity, reArrangeDistance, reArrangeEdge]
	# v = raw_input('try start-end connection? (y/n):\n')
	v = 'y'
	if v == 'y':
		PLNMRPathStartToTarget = PLNonMonotoneReArrangementSearchPrimitive(initCfgObjList, refCfgObjList, objectList, area, robot1, transitTransferRRTGraphParameter, env)
		if PLNMRPathStartToTarget is not 'null':
			reArrangeConnectivity[0][1] = 1.0
			reArrangeConnectivity[1][0] = 1.0
			# test 
			# pdb.set_trace() #breakpoint	
			setObjListTransformFromCfgList(PLNMRPathStartToTarget[0][0], objectList, area)
			reArrangeGraph[3].append([0, 1, PLNMRPathStartToTarget])
			# pathExecutionWithArm(PLNMRPathStartToTarget, objectList, robot1, env, transitTransferRRTGraphParameter, area)
			# pdb.set_trace() #breakpoint	

	numOfNeighbors = 5
	pathFoundInGraph = findPathReArrangementPRM(reArrangeGraph, initCfgObjList, refCfgObjList)
	while pathFoundInGraph is 'null':
		[newNode, nodeStatus] = sampleArrangementPRM(initCfgObjList, refCfgObjList, objectList, area, env)
		# pdb.set_trace()
		[reArrangeGraph, newNodeIndex]  = addNodeToReArrangementPRMGraph(reArrangeGraph, newNode, nodeStatus, area)
		closestNodeIndex = giveClosestNodeIndexReArrangementPRMGraph(reArrangeGraph, newNodeIndex, numOfNeighbors)
		for i_cfg in range(shape(closestNodeIndex)[0]):
			# pdb.set_trace()
			startCfgObjList = reArrangeGraph[0][newNodeIndex]
			targetCfgObjList = reArrangeGraph[0][closestNodeIndex[i_cfg]]
			initTransformObjList = giveTransformObjList(objectList) # save initial transformation
			PLNMRPath = PLNonMonotoneReArrangementSearchPrimitive(startCfgObjList, targetCfgObjList, objectList, area, robot1, transitTransferRRTGraphParameter, env)
			setTransformObjList(objectList, initTransformObjList) # restore previous transformation
			if PLNMRPath is not 'null':
				oneEdge = [newNodeIndex, closestNodeIndex[i_cfg], PLNMRPath]
				reArrangeGraph[1][newNodeIndex][closestNodeIndex[i_cfg]] = 1.0
				reArrangeGraph[1][closestNodeIndex[i_cfg]][newNodeIndex] = 1.0
				reArrangeGraph[3].append(oneEdge)
		# pdb.set_trace()
		pathFoundInGraph = findPathReArrangementPRM(reArrangeGraph, initCfgObjList, refCfgObjList)
	# return pathFoundInGraph
	timeCalcEnd = datetime.now()
	durationCalc1 = timeCalcEnd - timeCalcBegin
	print str(durationCalc1.total_seconds())
	print str(pathFoundInGraph)
	# pdb.set_trace() # breakpoint

	totalPathWithArm = totalPathExecutionWithArm(pathFoundInGraph, reArrangeGraph, objectList, robot1, area, transitTransferRRTGraphParameter, 'noAnimation', env)
	# pdb.set_trace() #breakpoint
	print 'planner finished'
	return totalPathWithArm

####################################################################################################
"""
	pdb.set_trace() # breakpoint
	# PLNMRPath = PLNonMonotoneReArrangementSearch(0, objIndexRemaining, newNode, cfg_Loop, objectList, area, env)
	initTransformObjList = giveTransformObjList(objectList) # save initial transformation
	PLNMRPathPrimitive1 = PLNonMonotoneReArrangementSearchPrimitive(initCfgObjList, newNode, objectList, area, transitTransferRRTGraphParameter, env)
	setTransformObjList(objectList, initTransformObjList) # restore previous transformation
	#initTransformObjList = giveTransformObjList(objectList) # save initial transformation
	PLNMRPathPrimitive2 = PLNonMonotoneReArrangementSearchPrimitive(newNode, refCfgObjList, objectList, area, transitTransferRRTGraphParameter, env)
	setTransformObjList(objectList, initTransformObjList) # restore previous transformation


	pathExecution(PLNMRPathPrimitive1, objectList, env, area)
	time.sleep(1)
	pathExecution(PLNMRPathPrimitive2, objectList, env, area)





	initTransformObjList = giveTransformObjList(objectList) # save initial transformation
	PLNMRPathPrimitive = PLNonMonotoneReArrangementSearchPrimitive(initCfgObjList, refCfgObjList, objectList, area, transitTransferRRTGraphParameter, env)
	setTransformObjList(objectList, initTransformObjList)

	pathExecution(PLNMRPathPrimitive, objectList, env, area)
	pdb.set_trace() #breakpoint	





#############################################################################################################################
# Yue's algorithm
#############################################################################################################################

# generate action sequence
# seqActions: object, targetPosOri(0 is goal), randX, randY
initTransformObjList = giveTransformObjList(objectList)
# pdb.set_trace() #breakpoint
goalTransformObjList = giveTransformObjList(refObjectList)
seqActions = initSeqAct(objectList, refObjectList, env)
seqActHist = []
# seqActHist = seqActHistAppend(seqActions, seqActHist)
# copySeqactions = seqActions[:][:]
seqActHist.append(seqActions.copy())
#print "seqActHist: "
#print seqActHist
seqActSuccs = False
iterationNum = 0
while(seqActSuccs is False):
	iterationNum = iterationNum + 1
	# pdb.set_trace() #breakpoint
	print seqActions

	[seqActSuccs, stepNumberInSeq, objCrrtForCllsn] = checkSeqActionCollision(objectList, refObjectList, refCfgObjList, initCfgObjList, seqActions, onlyPushObjEnum, area, env)
	print "seqActSuccs: " + str(seqActSuccs)
	print "stepNumberInSeq: " + str(stepNumberInSeq)
	print "objCrrtForCllsn: " + str(objCrrtForCllsn)
	print "iteration number: " + str(iterationNum)
	pdb.set_trace() #breakpoint
	if seqActSuccs is False:
		seqActions = tryReorderSeqAct(seqActions, stepNumberInSeq, objCrrtForCllsn)
		
		#print "seqActions: "
		#print seqActions
		#print "seqActHist: "
		#print seqActHist
		
		# pdb.set_trace() #breakpoint	
		if checkIfInHistory(seqActions,seqActHist): # there is a cyclic collision
		# if not all(seqActions-seqActHist): # there is a cyclic collision
			# check cyclic
			seqActions = addActRmvCycCllsn(seqActions, stepNumberInSeq, objectList, refCfgObjList, area, env)
		else:
			# copySeqactions = seqActions[:]
			seqActHist.append(seqActions.copy())

			#seqActHist = seqActHistAppend(seqActions, seqActHist)
if (seqActSuccs is True):
	pdb.set_trace() #breakpoint

	moveAnimFromSeqActions(objectList, refObjectList, seqActions, initCfgObjList, refCfgObjList, env)
pdb.set_trace() #breakpoint


# h4,h5,h6= PlotFrame(env, GRightInBase, 0.1)

# RRT
Tree = initCfgObjList
backTrackingVec = ['None']
TreeNorm = normalizeNode(node = Tree, area = area)
[randNode, randNodeNorm] = sampleNodeNorm(objectList)
TreeNode = extendTreeNode(Tree, TreeNorm, randNode, randNodeNorm)


area[0] = env.GetBodies()[tableEnum1].GetTransform()[0,3]
area[1] = env.GetBodies()[tableEnum1].GetTransform()[1,3]
area[2] = 0.8
area[3] = 0.8
"""