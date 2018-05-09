#!/usr/bin/env python

"""
@ file results.py
@ author Guelfi_Serena,Garello_Luca

@ The result node merge informations from merge and the slowest informations from pit. It matches all the detected
@ objects and writes it on the ontology.
@ The node matches objects from mergee and pit checking the id number of the object and replace x,y,z coordinates from merger with
@ the estimated coordinates of the centroid returned by pit. Shape and properties related to it are added. The node use a class
@ 'Object' in order to keep in memory the old state and use it as reference to check if properties are changed o not.
@ For each object is checked :
@ if it exist on the ontology: in negative case is added,
@ if its properties are changed: is checked if the change is relevant using a threshold and in positive case are replaced
@ if the shape is changed all the coefficient related to the shape are deleted and added the new one related to the new shape
@ In the library 'result_manipulation' are provided all the functions used by this node.

@ NB! All the objects detected during the execution are saved on the ontology;
@ brief is possible to modify the code in order to delete the individual or its properties if the object is no more detected
"""

import rospy
import os
from pitt_msgs.msg import TrackedShapes
from pitt_msgs.msg import TrackedShape
from tf_msgs.msg import Results
from armor_api.armor_client import ArmorClient
from armor_msgs.srv import ArmorDirective, ArmorDirectiveList, ArmorDirectiveListRequest
from armor_msgs.msg import _ArmorDirectiveReq
from objects import Object
from os.path import dirname, realpath
from result_manipulation import ResultManipulation
import numpy as np
import sys
import time



# INITIALIZE REFERENCE

path= dirname(realpath(__file__))
path = path + "/../../ontology/"



class results():

	## get param from the launch file for the threshold of the error in comparison for values
	threshold = rospy.get_param('/threshold_ontology_error')
	##open the library for the manipulation of this class
	res = ResultManipulation(threshold)
	#keep memory of the written individuals
	objects = []
	tensorlabel = []
	mergeid = []


	def __init__(self):

		##subscribe to the merger node on the results topic
		self._sub = rospy.Subscriber('/results', Results, self.callbackmerge, queue_size=1)
		##subscribe to the ransac segmentation node on the /ransac_segmentation/trackedShapes topic
		self._sub = rospy.Subscriber('/ransac_segmentation/trackedShapes' , TrackedShapes, self.callbackpit, queue_size=1)



	def callbackmerge(self,mergermsg):
		""" read the message 'Results' received from merger node and save data on memory"""
		self.mergeid = mergermsg.object_id
		self.tensorlabel = mergermsg.label

	def callbackpit(self,TrackedShapes_msg):
		""" read messages received from pit containig the slowest informations related to the shape of the object.
		 	The function merges these informations with the other received from merger node and write them in the ontology.
		 	It saves all individuals in a variable 'object' that keep in memory the old state
		"""
		lentensor=len(self.mergeid)
		lenpit=len(TrackedShapes_msg.tracked_shapes)
		if (lenpit>0 and lentensor>0):
			for i in range(0,lenpit) :
				for j in range(0,lentensor):      
					if (TrackedShapes_msg.tracked_shapes[i].object_id == self.mergeid[j]):
						match=0
						for k in range (0,len(self.objects)): 
							if (self.objects[k].id == self.mergeid[j]):
								match = 1

								self.objects[k].label = self.res.check_property(self.mergeid[j],"has-label","STRING",self.tensorlabel[j],self.objects[k].label)
								self.objects[k].Xcenter =self.res.check_property(self.mergeid[j], "has-geometric_centerX", "FLOAT",
													TrackedShapes_msg.tracked_shapes[i].x_est_centroid,self.objects[k].Xcenter)
								self.objects[k].Ycenter = self.res.check_property(self.mergeid[j], "has-geometric_centerY", "FLOAT",
													TrackedShapes_msg.tracked_shapes[i].y_est_centroid,self.objects[k].Ycenter)
								self.objects[k].Zcenter = self.res.check_property(self.mergeid[j], "has-geometric_centerZ", "FLOAT",
													TrackedShapes_msg.tracked_shapes[i].z_est_centroid,self.objects[k].Zcenter)	
								#check if the shape is changed
								if (self.objects[k].shape != TrackedShapes_msg.tracked_shapes[i].shape_tag):
									# if the shape is changed, also relative parameters are changed
									#delete old state and create a new one with new parameters
									self.res.remove_coefficients(self.objects[k])
									self.objects[k].delete_shapes_coefficients() 
									self.objects[k].shape = TrackedShapes_msg.tracked_shapes[i].shape_tag 
									self.res.add_coefficients(TrackedShapes_msg.tracked_shapes[i].coefficients,self.objects[k])


								else:
									self.res.check_properties_for_shapes(self.objects[k],TrackedShapes_msg.tracked_shapes[i].coefficients)
									
								print "UPDATING OBJECT",self.tensorlabel[j],"	SHAPE:",TrackedShapes_msg.tracked_shapes[i].shape_tag,"	ID:",self.mergeid[j]							
													
						#if nothing has been found or if the vector is empty, a new individual is created in the ontology
						#and the object is added to the vector
						if (match==0 or len(self.objects)==0):
							print "NEW OBJECT DETECTED", self.tensorlabel[j]
							##create a new object
							individual = Object()
							##init common properties for each shape
							individual.init_common_value(TrackedShapes_msg.tracked_shapes[i].object_id,self.tensorlabel[j],TrackedShapes_msg.tracked_shapes[i].shape_tag,
														 TrackedShapes_msg.tracked_shapes[i].x_est_centroid,TrackedShapes_msg.tracked_shapes[i].y_est_centroid,
														 TrackedShapes_msg.tracked_shapes[i].z_est_centroid)
														 
							self.res.add_new_individual_and_properties(individual)
							#add coefficients informations
							self.res.add_coefficients(TrackedShapes_msg.tracked_shapes[i].coefficients,individual)
							#add the new object in memory
							self.objects.append(individual)
			client.utils.apply_buffered_changes()
			client.utils.sync_buffered_reasoner()
			client.utils.save_ref_with_inferences(path + "empty-scene.owl")		

	def main(self):
		rospy.spin()

if __name__ == '__main__':

	rospy.init_node('results')
	#wait for the execution of ARMOR
	while True:
		try:
			client = ArmorClient("client", "reference")
			# initializing with buffered manipulation and reasoning
			client.utils.load_ref_from_file(path + "empty-scene.owl",
							"http://www.semanticweb.org/emaroLab/luca-buoncompagni/sit",True, "PELLET", True, False)
			client.utils.mount_on_ref()
			client.utils.set_log_to_terminal(True)
			break
		except Exception:
			time.sleep(2)

	results = results()
	results.main()
