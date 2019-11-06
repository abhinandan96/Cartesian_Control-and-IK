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
from threading import Thread, Lock
import time


'''This is a class which will perform both cartesian control and inverse
   kinematics'''
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
	self.qc = []
	

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
	    #print(current_joint)
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link
	#print(self.joint_axes)

    '''This is the callback which will be executed when the cartesian control
       recieves a new command. The command will contain information about the
       secondary objective and the target q0. At the end of this callback, 
       you should publish to the /joint_velocities topic.'''
    def get_cartesian_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR CARTESIAN CONTROL HERE
	#print(self.q_current)
	joint_transforms, b_T_ee = self.forward_kinematics(self.q_current)
	J = self.get_jacobian(b_T_ee, joint_transforms)
	q = [command.x_target.rotation.x, command.x_target.rotation.y, command.x_target.rotation.z, command.x_target.rotation.w]
	trans = numpy.dot(tf.transformations.translation_matrix((command.x_target.translation.x, command.x_target.translation.y, command.x_target.translation.z)), tf.transformations.quaternion_matrix(q))
	#print(trans)
	
	eetd = numpy.dot(tf.transformations.inverse_matrix(b_T_ee),trans)
	#print(eetd)
	angle, axis = self.rotation_from_matrix(eetd)
	changeor = numpy.dot(axis,angle)
	xdes = numpy.concatenate((numpy.transpose(eetd[:3,3]), changeor), axis = 0)
	#print(xdes)
	qdot = numpy.dot(numpy.linalg.pinv(J,0.01), numpy.transpose(xdes))
	com = numpy.array([command.x_target.translation.x,command.x_target.translation.y,command.x_target.translation.z,command.x_target.rotation.x,command.x_target.rotation.y,command.x_target.rotation.z,command.x_target.rotation.w])	
	#print(qdot)
	if self.num_joints == 7:
		qsec = numpy.array([(command.q0_target - qdot[0]) * 3,0,0,0,0,0,0])
		qnull = numpy.dot((numpy.identity(self.num_joints) - numpy.dot(numpy.linalg.pinv(J), J)), numpy.transpose(qsec))
		qdot = qdot + qnull
	
	
	self.joint_velocity_msg.name = self.joint_names
	self.joint_velocity_msg.velocity = qdot
	self.velocity_pub.publish(self.joint_velocity_msg)
        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callback will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def get_jacobian(self, b_T_ee, joint_transforms):
        J = numpy.zeros((6,self.num_joints))
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR ASSEMBLING THE CURRENT JACOBIAN HERE
	for i in range(0, self.num_joints):
		jtransee = numpy.dot(tf.transformations.inverse_matrix(joint_transforms[i]), b_T_ee)
		#print(i)
		#print('bte')
		#print(b_T_ee)
		#print('jtb')
		#print(tf.transformations.inverse_matrix(joint_transforms[i]))
		#print('etj')
		#print(tf.transformations.inverse_matrix(jtransee))
		#print('jte')
		#print(jtransee)
		ttrans = jtransee[:3,3]
		skewtrans = numpy.array([[0, -ttrans[2], ttrans[1]], [ttrans[2], 0, -ttrans[0]], [-ttrans[1], ttrans[0], 0]])
		rtrans = jtransee[:3,:3]
		rtransinv = tf.transformations.inverse_matrix(rtrans)
		lef = numpy.concatenate(((rtransinv),numpy.array([[0,0,0], [0,0,0], [0,0,0]])), axis = 0)
		rig = numpy.concatenate((numpy.dot(-rtransinv, skewtrans),rtransinv), axis = 0)
		Vj = numpy.concatenate((lef,rig), axis = 1)
		axes = numpy.concatenate((numpy.array([0, 0, 0]), self.joint_axes[i]), axis = 0)
		J[:,i] = numpy.dot(Vj, numpy.transpose(axes))
		#print(Vj)
		#print(i)
		#print(axes)
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

	for j in range(0,10):
		self.qc = []
		for i in range(0,self.num_joints):
			self.qc.append(0.001 * (random.randint(-1000,1000)))
		a = 0
		at = time.time()
		while a < 10:			
			joint_transforms, b_T_ee = self.forward_kinematics(self.qc)
			J = self.get_jacobian(b_T_ee, joint_transforms)
			q = [command.rotation.x, command.rotation.y, command.rotation.z, command.rotation.w]
			trans = numpy.dot(tf.transformations.translation_matrix((command.translation.x, command.translation.y, command.translation.z)), tf.transformations.quaternion_matrix(q))
			eetd = numpy.dot(tf.transformations.inverse_matrix(b_T_ee),trans)
			angle, axis = self.rotation_from_matrix(eetd)
			changeor = numpy.dot(axis,angle)
			xdes = numpy.concatenate((numpy.transpose(eetd[:3,3]), changeor), axis = 0)
			if xdes.all() == 0:
				qdot = numpy.dot(numpy.linalg.pinv(J,0.01), numpy.transpose(xdes))
				q = []
				for i in range(0,self.num_joints):
					q.append(qdot[i])
				for i in range(0,self.num_joints):
					self.qc[i] = q[i] + self.qc[i]
				self.joint_command_msg.name = self.joint_names
				self.joint_command_msg.position = self.qc
				self.joint_command_pub.publish(self.joint_command_msg)
				break
			qdot = numpy.dot(numpy.linalg.pinv(J,0.01), numpy.transpose(xdes))
			q = []
			for i in range(0,self.num_joints):
				q.append(qdot[i])
			for i in range(0,self.num_joints):
				self.qc[i] = q[i] * 0.001 + self.qc[i]	
			self.joint_command_msg.name = self.joint_names
			self.joint_command_msg.position = self.qc
			self.joint_command_pub.publish(self.joint_command_msg)
			bt  = time.time()
			a = bt - at		
		
			
	
	
	

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
        self.mutex.release()


if __name__ == '__main__':
    rospy.init_node('cartesian_control_and_IK', anonymous=True)
    CCIK()
    rospy.spin()

