#!/usr/bin/env python

import math
import numpy
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from cartesian_control.msg import CartesianCommand
from urdf_parser_py.urdf import URDF
import random
import tf
import time
from threading import Thread, Lock

'''This is a class which will perform both cartesian control and inverse
   kinematics'''
v = 0
class CCIK(object):
    def __init__(self):
	#Load robot from parameter server
        self.robot = URDF.from_parameter_server()

	#Subscribe to current joint state of the robot
        rospy.Subscriber('/joint_states', JointState, self.get_joint_state)

	#This will load information about the joints of the robot
        self.num_joints = 0
        self.joint_names = []
        self.q_current = []
        self.joint_axes = []
        self.get_joint_info()

	#This is a mutex
        self.mutex = Lock()

	#Subscribers and publishers for for cartesian control
        rospy.Subscriber('/cartesian_command', CartesianCommand, self.get_cartesian_command)
        self.velocity_pub = rospy.Publisher('/joint_velocities', JointState, queue_size=10)
        self.joint_velocity_msg = JointState()

        #Subscribers and publishers for numerical IK
        rospy.Subscriber('/ik_command', Transform, self.get_ik_command)
        self.joint_command_pub = rospy.Publisher('/joint_command', JointState, queue_size=10)
        self.joint_command_msg = JointState()

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link

    '''This is the callback which will be executed when the cartesian control
       receives a new command. The command will contain information about the
       secondary objectscratch_6ive and the target q0. At the end of this callback, 
       you should publish to the /joint_velocities topic.'''
    def get_cartesian_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR CARTESIAN CONTROL HERE
        joint_values = self.q_current
        #print(self.q_current)
        joint_transforms, b_T_ee = self.forward_kinematics(joint_values)
        a = numpy.zeros((3,1))
        b = numpy.zeros((3,1))
        b_T_edes = tf.transformations.quaternion_matrix([command.x_target.rotation.x,command.x_target.rotation.y
                                                            ,command.x_target.rotation.z,command.x_target.rotation.w])
        b_T_edes[0,3] = command.x_target.translation.x
        b_T_edes[1,3] = command.x_target.translation.y
        b_T_edes[2,3] = command.x_target.translation.z
        ee_T_edes = numpy.dot(tf.transformations.inverse_matrix(b_T_ee),b_T_edes)
        a[:,0] = ee_T_edes[0:3,3]
        angle, axis = self.rotation_from_matrix(ee_T_edes)
        b[:,0] = numpy.dot(axis,angle)
        delx = numpy.concatenate((a,b),axis=0)
        gain = 1.1
        v_ee = numpy.dot(delx,gain)
        J = self.get_jacobian(b_T_ee, joint_transforms)
        q_des = numpy.dot(numpy.linalg.pinv(J, 0.01), v_ee)
        if 1:
            q_null = numpy.zeros((self.num_joints,1))
            q_sec = numpy.zeros((self.num_joints,1))
            p = 3
            q_sec[0] = p*(command.q0_target-q_des[0])
            I = numpy.identity((self.num_joints))
            q_null = numpy.dot((I-numpy.dot(numpy.linalg.pinv(J),J)), q_sec)
            q_des = q_des+q_null
            """global v
            if v ==0:
                v+=1
                print(q_null)
                print(q_sec)
                print(q_des)"""
        #print(q_des)
        #print(ab)
        print(q_des)
        self.joint_velocity_msg.name = self.joint_names
	print(self.joint_names)
        self.joint_velocity_msg.velocity = q_des[0]
        self.joint_command_pub.publish(self.joint_velocity_msg)
        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callback will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def get_jacobian(self, b_T_ee, joint_transforms):
        J = numpy.zeros((6,self.num_joints))
        l = self.num_joints
        global v
        j_T_ee = numpy.zeros ((4,4))
        ee_R_j = numpy.zeros ((3))
        ee_T_j = []
        Vj = []
        xj = []
        skew = numpy.zeros((3,3))
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR ASSEMBLING THE CURRENT JACOBIAN HERE
        for i in range (len(joint_transforms)):
            """jtob = tf.transformations.inverse_matrix(joint_transforms[i])
            j_T_ee = numpy.dot(jtob,b_T_ee)
            ee_T_j = tf.transformations.inverse_matrix(j_T_ee)
            skew = [[0,j_T_ee[2][2],-j_T_ee[1][2]],[-j_T_ee[2][2],0,j_T_ee[0][2]],[j_T_ee[1][2],-j_T_ee[0][2],0]]
            skew = numpy.dot(ee_T_j[0:3,0:3],skew)
            Vj = numpy.bmat([[ee_T_j[0:3,0:3],skew],[numpy.zeros((3,3)),ee_T_j[0:3,0:3]]])
            a = numpy.concatenate(([0.0, 0.0, 0.0],[self.joint_axes[i]]), axis = None)
            #print(a)"""
            j_T_ee = numpy.dot(tf.transformations.inverse_matrix(joint_transforms[i]),b_T_ee)
            #ee_R_j = j_T_ee[0:3][0:3]
            ee_R_j = numpy.transpose(j_T_ee[0:3,0:3])
            #print(ee_R_j)
            #print(j_T_ee)
            j_trans_ee = j_T_ee[0:3,3]
            skew = numpy.array([[0,j_trans_ee[2],-j_trans_ee[1]],[-j_trans_ee[2], 0,j_trans_ee[0]],[j_trans_ee[1],-j_trans_ee[0],0]])
            #print(j_trans_ee)
            #print(skew)
            skew = numpy.dot(ee_R_j,skew)
            #print(ee_R_j)
            #print(skew)
            #Vj = numpy.bmat([[ee_T_j[0:3,0:3],skew],[numpy.zeros((3,3)),ee_T_j[0:3,0:3]]])
            Vj = numpy.bmat([[ee_R_j,skew],[numpy.zeros((3,3)),ee_R_j]])
            #print(Vj)
            #print(ee_R_j)
            #skew = numpy.dot(ee_R_j,)
            #Vj = numpy.bmat([])
            axis = numpy.concatenate(([0.0, 0.0, 0.0],[self.joint_axes[i]]), axis = None)
            J[:,i]=numpy.dot(Vj,axis)
            """if self.joint_axes[i][2] == 1.0:
                for j in range(6):
                    J[j][i] = Vj[j, 5]
            elif self.joint_axes[i][2] == -1.0:
                for j in range(6):
                    J[j][i] = -Vj[j, 5]
            elif self.joint_axes[i][1] == 1.0:
                for j in range(6):
                    J[j][i] = Vj[j, 4]
            elif self.joint_axes[i][1] == -1.0:
                for j in range(6):
                    J[j][i] = -Vj[j, 4]
            elif self.joint_axes[i][0] == 1.0:
                for j in range(6):
                    J[j][i] = Vj[j, 3]
            elif self.joint_axes[i][0] == -1.0:
                for j in range(6):
                    J[j][i] = -Vj[j, 3]
"""
        #if v == 0:
            #print(j_T_ee[0][2])
            #v=v+1
            #print(xj[0][:,5])
            #print(xj[1])
            #print(J)
        #--------------------------------------------------------------------------
        return J

    '''This is the callback which will be executed when the inverse kinematics
       recieve a new command. The command will contain information about desired
       end effector pose relative to the root of your robot. At the end of this
       callback, you should publish to the /joint_command topic. This should not
       search for a solution indefinitely - there should be a time limit. When
       searching for two matrices which are the same, we expect numerical
       precision of 10e-3.'''
    def get_ik_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR INVERSE KINEMATICS HERE
        b_T_edes = tf.transformations.quaternion_matrix([command.rotation.x, command.rotation.y
                                                          , command.rotation.z, command.rotation.w])
        b_T_edes[0, 3] = command.translation.x
        b_T_edes[1, 3] = command.translation.y
        b_T_edes[2, 3] = command.translation.z
        l = self.num_joints
        joint_values = numpy.zeros((l, 1))
        for i in range(l):
            joint_values[i] = random.randint(0, 1000)*0.001
        t = 0
        a = numpy.zeros((3, 1))
        b = numpy.zeros((3, 1))
        c = 0
        c = time.time()
        while t<5:
            joint_transforms, b_T_ee = self.forward_kinematics(joint_values)
            ee_T_b = tf.transformations.inverse_matrix(b_T_ee)
            ee_T_eed = numpy.dot(ee_T_b,b_T_edes)
            a[:, 0] = ee_T_eed[0:3, 3]
            angle, axis = self.rotation_from_matrix(ee_T_eed)
            b[:, 0] = numpy.dot(axis, angle)
            delx = numpy.concatenate((a, b), axis=0)
            #print(b,delx)
            J = self.get_jacobian(b_T_ee,joint_transforms)
            Jp = numpy.linalg.pinv(J, 0.01)
            delq = numpy.dot(Jp,delx)
            joint_values = joint_values + delq
            self.joint_command_pub.name = self.joint_names
            self.joint_command_msg.position = joint_values
            self.velocity_pub.publish(self.joint_command_msg)
            d = time.time()
            t = d-c
        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This function will return the angle-axis representation of the rotation
       contained in the input matrix. Use like this: 
       angle, axis = rotation_from_matrix(R)'''
    def rotation_from_matrix(self, matrix):
        R = numpy.array(matrix, dtype=numpy.float64, copy=False)
        R33 = R[:3, :3]
        # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, W = numpy.linalg.eig(R33.T)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        axis = numpy.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, Q = numpy.linalg.eig(R)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        # rotation angle depending on axis
        cosa = (numpy.trace(R33) - 1.0) / 2.0
        if abs(axis[2]) > 1e-8:
            sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
        elif abs(axis[1]) > 1e-8:
            sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
        else:
            sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
        angle = math.atan2(sina, cosa)
        return angle, axis

    '''This is the function which will perform forward kinematics for your 
       cartesian control and inverse kinematics functions. It takes as input
       joint values for the robot and will return an array of 4x4 transforms
       from the base to each joint of the robot, as well as the transform from
       the base to the end effector.
       Usage: joint_transforms, b_T_ee = self.forward_kinematics(joint_values)'''
    def forward_kinematics(self, joint_values):
        joint_transforms = []

        link = self.robot.get_root()
        T = tf.transformations.identity_matrix()

        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]

            T_l = numpy.dot(tf.transformations.translation_matrix(joint.origin.xyz), tf.transformations.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]))
            T = numpy.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)
                T_j = tf.transformations.rotation_matrix(joint_values[q_index], numpy.asarray(joint.axis))
                T = numpy.dot(T, T_j)

            link = next_link
        return joint_transforms, T #where T = b_T_ee

    '''This is the callback which will recieve and store the current robot
       joint states.'''
    def get_joint_state(self, msg):
        self.mutex.acquire()
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])
        #print(self.q_current)
        self.mutex.release()


if __name__ == '__main__':
    rospy.init_node('cartesian_control_and_IK', anonymous=True)
    CCIK()
    rospy.spin()
