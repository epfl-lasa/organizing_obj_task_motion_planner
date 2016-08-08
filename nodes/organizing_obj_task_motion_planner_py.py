#!/usr/bin/env python
# Read on the joint_state_imp topic, and for each message received, answer with des velocities

import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
import time
import pdb
import sys
sys.path.insert(0, sys.path[0] + '/OpenRaveSim/python/Yue')
from Yues_Experiment_Demo import * 
from numpy import linalg as LA

from kuka_fri_bridge.msg                      import JointStateImpedance
from sensor_msgs.msg                          import JointState
from organizing_obj_task_motion_planner.msg   import ObjectPoses
from organizing_obj_task_motion_planner.msg   import HandReady
from organizing_obj_task_motion_planner.msg   import HandCommand
from bhand_controller.srv                     import Actions, SetControlMode

class passAJointValueSeq(object):
    """convert kuka_fri_bridge/JointStateImpedance message to std_msgs/Float32MultiArray for position controller simulator"""
    def __init__(self, jointValueSeq):
        # pdb.set_trace() #breakpoint
        self.totalActionNum = np.shape(jointValueSeq)[0]
        self.handReady = True
        self.ifFinished = False
        self.jointValueSeq = jointValueSeq
        self.crrtIndexAction = 0
        # self.pos_input_topic_type = JointStateImpedance
        self.input_pos_msg = JointStateImpedance()
        self.handCommand_msg = HandCommand()
        self.handJSCommand_msg = JointState()
        self.passJs_pub = rospy.Publisher("/KUKA/joint_imp_cmd", numpy_msg(JointStateImpedance), queue_size = 3, tcp_nodelay = True, latch = False)
        self.js_sub = rospy.Subscriber("/joint_states", numpy_msg(JointState), self.js_cb, queue_size=3, tcp_nodelay=True)
        self.handReady_sub = rospy.Subscriber("/BarrettHandReady", numpy_msg(HandReady), self.handReady_cb, queue_size=1, tcp_nodelay=True)
        self.handCommand_pub = rospy.Publisher("/BarrettHandCommand", numpy_msg(HandCommand), queue_size = 1, tcp_nodelay = True, latch = False)
        self.handJointStateCommand = rospy.Publisher("/bhand_node/command", numpy_msg(JointState), queue_size = 1, tcp_nodelay = True, latch = False)
        self.ifSendedCommandBHand = False
        self.printedFinished = False
        # pdb.set_trace() #breakpoint
        # rospy_rate = rospy.Rate(20)
        # rospy.loginfo("The new position: %d" % (position[0]))       
    def handReady_cb(self, msg):
        self.handReady = msg.handReady
        # pdb.set_trace() #breakpoint
        self.crrtIndexAction = self.crrtIndexAction + 1
        # if self.crrtIndexAction == np.shape(self.jointValueSeq)[0]:
            # pdb.set_trace() #breakpoint

    def js_cb(self, msg):
        # pdb.set_trace() #breakpoint
        self.refPositionCrrt = self.jointValueSeq[self.crrtIndexAction]
        if np.shape(msg.position)[0] ==8:
            pdb.set_trace() #breakpoint
        if self.refPositionCrrt[0] == 5: # close fingers
            # pdb.set_trace() #breakpoint
            self.handCommand_msg.handCommand = 5
            self.handCommand_pub.publish(self.handCommand_msg)
            self.handReady = False
            if self.ifSendedCommandBHand == False:
                self.ifSendedCommandBHand = True
                self.sendCommandBHand_client('close', self.refPositionCrrt)
        elif self.refPositionCrrt[0] == 10: # open fingers
            self.handCommand_msg.handCommand = 10
            self.handCommand_pub.publish(self.handCommand_msg)
            self.handReady = False
            if self.ifSendedCommandBHand == False:
                self.ifSendedCommandBHand = True
                self.sendCommandBHand_client('open', self.refPositionCrrt)
        elif self.refPositionCrrt[0] == 15: # finished
            # pdb.set_trace()
            self.ifFinished = True
            self.refPositionCrrt = msg.position
            if self.printedFinished == False:
                print 'finished'
                self.printedFinished = True
        # print str(self.crrtIndexAction) + '/' + str(np.shape(self.jointValueSeq)[0])
        diffPosition = 1
        try:
            diffPosition = self.refPositionCrrt - msg.position
        except ValueError:
            print self.refPositionCrrt
            print 'msg'
            print msg.position
            # pdb.set_trace()
        
        self.input_pos_msg.position = self.refPositionCrrt
        # rospy_rate = rospy.Rate(20)
        if self.handReady == False and msg.header.seq % 100 == 0:
            print  'waiting for hand to be ready at sequence: ' + str(self.crrtIndexAction) + '/' + str(np.shape(self.jointValueSeq)[0])
        if (LA.norm(diffPosition) > 0.10) and (not rospy.is_shutdown()) and self.handReady:
            self.passJs_pub.publish(self.input_pos_msg)
        if (LA.norm(diffPosition) < 0.10) and (self.crrtIndexAction < self.totalActionNum-1) and not self.ifFinished:
            self.crrtIndexAction = self.crrtIndexAction + 1
        """
        if self.crrtIndexAction != self.totalActionNum-1:
            print 'refPosition:' + str(self.refPositionCrrt)
            print 'diffPosition:' + str(LA.norm(diffPosition))
        """

    def sendCommandBHand_client(self, sendCommandBHand, refPositionCrrt):
        rospy.wait_for_service('bhand_node/actions')
        try:
            sendCommandBHandProxy = rospy.ServiceProxy('bhand_node/actions', Actions)
            time.sleep(3)
            if sendCommandBHand == 'close':
                print 'hand closing'
                f1RefPos = refPositionCrrt[1]
                f2RefPos = refPositionCrrt[2]
                f3RefPos = refPositionCrrt[3]
                self.handJSCommand_msg.name = ['bh_j11_joint','bh_j32_joint','bh_j12_joint','bh_j22_joint']
                self.handJSCommand_msg.position = [0,f3RefPos, f2RefPos, f1RefPos]
                self.handJSCommand_msg.velocity = [0.1, 0.1, 0.1, 0.1]
                self.handJSCommand_msg.effort = [0,0,0,0]
                self.handJointStateCommand.publish(self.handJSCommand_msg)
                # resp1 = sendCommandBHandProxy(2)
            elif sendCommandBHand == 'open':
                print 'hand opening'
                self.handJSCommand_msg.name = ['bh_j11_joint','bh_j32_joint','bh_j12_joint','bh_j22_joint']
                self.handJSCommand_msg.position = [0,0.1, 0.1, 0.1]
                self.handJSCommand_msg.velocity = [0.1, 0.1, 0.1, 0.1]
                self.handJSCommand_msg.effort = [0,0,0,0]
                self.handJointStateCommand.publish(self.handJSCommand_msg)
                # resp1 = sendCommandBHandProxy(3)
            elif sendCommandBHand == 'init':
                print 'hand initialising'
                resp1 = sendCommandBHandProxy(1)
            time.sleep(5)
            self.handReady = True
            self.ifSendedCommandBHand = False
            self.crrtIndexAction = self.crrtIndexAction + 1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            pdb.set_trace()

    def stop(self):
        '''Stop the object'''
        self.passJs_pub.unregister()
        self.js_sub.unregister()
        self.handReady_sub.unregister()
        self.handCommand_pub.unregister()

class taskMotionPlanning(object):
    def __init__(self):
        self.sendCommandBHand_clientTaskLevel('init')
        self.jointStateCrrt = np.zeros(7)
        self.objectTargetPoseListener = rospy.Subscriber("/OBJECT/target_poses", numpy_msg(ObjectPoses), self.newTargetPose_cb, queue_size=3, tcp_nodelay=True)
        self.objectPoseListener = rospy.Subscriber("/OBJECT/current_poses", numpy_msg(ObjectPoses), self.updatePose_cb, queue_size=3, tcp_nodelay=True)        
        self.plannerJointStateListener = rospy.Subscriber("/joint_states", numpy_msg(JointState), self.plannerJointStateListener_cb, queue_size=3, tcp_nodelay=True)
        self.passingJointValue = []
        # self.objectNames = ['fork', 'kitchen_glass', 'kitchen_cup', 'mug']
        # self.crrtPoses = [-0.14571098, -0.05057669, 233.18074748, -0.33164334, -0.00400644, 212.02381944, -0.14137304, 0.21248677, 176.49369886, 0.02591017, 0.19922812, 136.21972607]
        # self.targetPoses = [-0.14002592, -1.53879333, 119.79488427, -0.13919334, -1.31785508, 82.33719603, -0.20101912, -1.71602603, 144.13363736, -0.00421196, -1.64668047, 39.73772348]
        self.objectNames = ['master_chef_coffee', 'red_metal_cup', 'red_metal_bowl']
        self.crrtPoses = [-0.25456, -0.14179, 0, -0.11312, 0.06053, 176.49369886, 0.00579, -0.09805, 136.21972607, -0.26054, 0.13388, 20]
        self.targetPoses = [-0.00924, -0.1255, 82.33719603, -0.20073, -0.08141, 144.13363736, -0.04941, 0.11, 39.73772348, -0.2369, 0.07923, 23]
        # rostopic pub -1 /OBJECT/target_poses organizing_obj_task_motion_planner/ObjectPoses '{poses: [-0.00924, -0.1255, 82.33719603, -0.20073, -0.08141, 144.13363736, -0.04941, 0.11, 39.73772348, -0.2369, 0.07923, 23]}'
        # self.crrtPoses = [-0.33164334, -0.00400644, 212.02381944, -0.14137304, 0.21248677, 176.49369886, 0.02591017, 0.19922812, 136.21972607]
        # self.targetPoses = [-0.13919334, -1.31785508, 82.33719603, -0.20101912, -1.71602603, 144.13363736, -0.00421196, -1.64668047, 39.73772348]
        # crrt      [-0.14571098, -0.05057669, 233.18074748, -0.33164334, -0.00400644, 212.02381944, -0.14137304, 0.21248677, 176.49369886, 0.02591017, 0.19922812, 136.21972607]
        # target    [-0.14002592, -0.03879333, 119.79488427, -0.13919334, 0.18214492, 82.33719603, -0.20101912, -0.21602603, 144.13363736, -0.00421196, -0.14668047, 39.73772348]
        # rostopic pub -1 /OBJECT/target_poses organizing_obj_task_motion_planner/ObjectPoses '{poses: [-0.13719507, 0.01681, 47.61236234, -0.28935386, -0.10644513, 188.54364701, -0.09866113, -0.16747361, 251.27810162, -0.30999605, 0.0611634, 176.64029439]}'
        # rostopic pub -1 /BarrettHandReady organizing_obj_task_motion_planner/HandReady '{handReady: True}'
        # rostopic echo /joint_states
        # quat = tf.transformations.quaternion_from_matrix(rotation)

    def sendCommandBHand_clientTaskLevel(self, sendCommandBHandTasklevel):
        rospy.wait_for_service('bhand_node/actions')
        try:
            sendCommandBHandTaskLevelProxy = rospy.ServiceProxy('bhand_node/actions', Actions)
            if sendCommandBHandTasklevel == 'init':
                resp1 = sendCommandBHandTaskLevelProxy(1)
            elif sendCommandBHandTasklevel == 'close':
                resp1 = sendCommandBHandTaskLevelProxy(2)
            elif sendCommandBHandTasklevel == 'open':
                resp1 = sendCommandBHandTaskLevelProxy(3)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            pdb.set_trace()


    def updatePose_cb(self, msg):
        self.objectNames = msg.objNames
        self.crrtPoses = msg.poses

    def newTargetPose_cb(self, msg):
        print 'new ref'
        # pdb.set_trace() # breakpoint
        if isinstance(self.passingJointValue, passAJointValueSeq):
            self.passingJointValue.stop()
            del self.passingJointValue
            print 'stop the old manipulation, start a new calculation #################################'

        # self.objectNames = msg.objNames
        self.targetPoses = msg.poses
        # pdb.set_trace() # breakpoint
        self.jointValueSeq = giveActionSeq(self.objectNames, self.crrtPoses, self.targetPoses, self.jointStateCrrt)
        # pdb.set_trace() # breakpoint
        self.passingJointValue = passAJointValueSeq(self.jointValueSeq)

    def plannerJointStateListener_cb(self, msg):
        self.jointStateCrrt = msg.position

    def stop(self):
        if isinstance(self.passingJointValue, passAJointValueSeq):
            self.passingJointValue.stop()
            print 'stop the old manipulation #################################################'
        self.objectPoseListener.unregister()
        self.plannerJointStateListener.unregister()
        self.objectTargetPoseListener.unregister()
"""
def giveActionSeq(paramA, paramB, paramC):
    if paramB[0] == 15:
        return [[0.29, -0.26, 0.11, -1.7, 0.96, 1.8, -2.43], [0.1, -0.32, 1, -0.5, 1, 2, 0], [5,5,5,5,5,5,5], [0.15, -1.2, 1.02, -0.65, 0.98, -0.15, 0.22]]
    else:
        return [[1.29, -0.26, 0.11, -1.7, 0.96, 1.8, -2.43], [-1.1, 1.32, -1, 0.5, -1, -2, 0.1], [5,5,5,5,5,5,5], [-0.15, 1.2, -1.02, 0.65, -0.98, -0.15, 0.22]]
""" 
def main():
    # absabs(1)
    # jointValueSeq = [[0.29, -0.26, 0.11, -1.7, 0.96, 1.8, -2.43], [0.1, -0.32, 1, -0.5, 1, 2, 0], [0.15, -1.2, 1.02, -0.65, 0.98, -0.15, 0.22]]
    rospy.init_node('organizing_obj_task_motion_planner', anonymous=True) #
    rospy.loginfo("%s: Starting" % (rospy.get_name())) # 
    
    # time.sleep(1)
    """
    s = raw_input('please let it move. (y/n)?')
    if s == 'y':
        position = [0.29, -0.26, 0.11, -1.7, 0.96, 1.8, -2.43]
    else:
        position = [0,0,0,0,0,0,0]
    """
    # pdb.set_trace() #breakpoint    
    # raw_input('Press any key to start.')
    taskMotionPlan = taskMotionPlanning()
    

    rospy.spin()
    rospy.loginfo("%s: Exiting" % (rospy.get_name()))
    taskMotionPlanning.stop()



if __name__ == '__main__':
    main()

