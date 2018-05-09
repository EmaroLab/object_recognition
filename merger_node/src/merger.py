#!/usr/bin/env python

"""
@file merger.py
@author Guelfi_Serena,Garello_Luca

@ merger node compares fastest informations from pit with informations from tensorflow
@ in order to match all the detected objects and merge all the informations.
@ The node compares x and y coordinates of each object in order to match them.
@ The threshold for the error is initialized in the launch file. 
@ Each object is written on the 'Result' message. On the are written:
@ x,y,z coordinates returned by pit, label and confidence , id of the object.
@ Label is set as unknown if the object isn't detected from tensorflow.
"""

import rospy
import os
from pitt_msgs.msg import ClustersOutput
from tf_msgs.msg import TensorOutput
from tf_msgs.msg import Results
import numpy as np
import sys
from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt



class merger():

	## get from the launch file the value of the threshold for the error between pit and tensorflow coordinates
	threshold = rospy.get_param('/threshold_error')


	def __init__(self):
		self.label = []
		self.Xtensor = []
		self.Ytensor = []
		##subscriber to tensorflow node on TensorOutput topic
		self._sub = rospy.Subscriber('/TensorOutput', TensorOutput, self.callbacktf, queue_size=1)
		##subscriber to the geometric tracker node on /geometric_tracker/trackedCluster topic
		self._sub = rospy.Subscriber('/geometric_tracker/trackedCluster', ClustersOutput, self.callbackpit, queue_size=1)
		##publisher to the result node on results topic
		self._pub = rospy.Publisher('/results', Results, queue_size=1)


	def callbacktf(self,TensorOutput_msg):
		"""callback for the messages received from tensorflow. It reads the variables contained in the message
			and save them into memory"""

		self.label = []
		self.label = TensorOutput_msg.result
		self.Xtensor= TensorOutput_msg.x
		self.Ytensor = TensorOutput_msg.y


	def callbackpit(self,ClustersOutput_msg):
		"""callback for messages received from pitt. Compares objects received from pit with objects detected from tensorflow.
		 	Check if they match and in that case merge the informations.
		 	All the detected objects are written on a message 'Results' that is published on /results topic."""

		# get the number of recognition
		npit = len(ClustersOutput_msg.cluster_objs)
		ntens = len(self.label)
		print "\nPit has detected",npit, "objects, Tensorflow",ntens,"objects\n"

		res=Results()
		#err_x=[]
		#err_y=[]
		if (npit>0 or ntens>0):
			for i in range(0,npit):
				found = 0
				for j in range(0,ntens):
					if ((abs(ClustersOutput_msg.cluster_objs[i].x_centroid - self.Xtensor[j]) < self.threshold) and
	                   (abs(ClustersOutput_msg.cluster_objs[i].y_centroid - self.Ytensor[j]) < self.threshold)):
						#data are written to the message
						res.label.append( self.label[j])
						res.x.append( ClustersOutput_msg.cluster_objs[i].x_centroid)
						res.y.append( ClustersOutput_msg.cluster_objs[i].y_centroid)
						res.z.append( ClustersOutput_msg.cluster_objs[i].z_centroid)
						res.object_id.append( ClustersOutput_msg.cluster_objs[i].shape_id)

						found = 1
						#compute and print the error
						#err_x.append(ClustersOutput_msg.cluster_objs[i].x_centroid - self.Xtensor[j])
						#err_y.append(ClustersOutput_msg.cluster_objs[i].y_centroid - self.Ytensor[j])


					#if they don't match items are written without some informations
				if 	(found	== 0):
					res.x.append( ClustersOutput_msg.cluster_objs[i].x_centroid)
					res.y.append( ClustersOutput_msg.cluster_objs[i].y_centroid)
					res.z.append( ClustersOutput_msg.cluster_objs[i].z_centroid)
					res.object_id.append( ClustersOutput_msg.cluster_objs[i].shape_id)
					res.label.append('unknown')


		if (len(res.x)>0):
			#publish the message
			self._pub.publish(res)


	def main(self):

		rospy.spin()

if __name__ == '__main__':

	rospy.init_node('merger')
	merger = merger()
	merger.main()
