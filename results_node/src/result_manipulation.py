#!/usr/bin/env python


"""
@file results_manipulation.py
@author Guelfi_Serena,Garello_Luca
@ It's a library for the manipulation of the class results. It contains functions to add,remove and check informations
@ to the ontology.
"""

import rospy
import os
from armor_api.armor_client import ArmorClient
from objects import Object
import sys


class ResultManipulation():
    def __init__(self, thrs):
        ##get from the class results the value of the threshold for the error that determines if the values are changed
        self.threshold = thrs
        ## client for the ontology
        self.client = ArmorClient("client", "reference")


    def add_new_individual_and_properties(self, object):
        """ add a new individual to the ontology if it doesn't exist and add commmon dataproperties

       @param object (class Object): individual that represent the old state
       """
        # add individual to the ontology

        properties = []
        self.client.manipulation.add_ind_to_class(str(object.id), "Objects")
        # add common information to the ontology
        properties.append(["has-label", "STRING", object.label])
        properties.append(["has-id", "FLOAT", str(object.id)])
        properties.append(["has-geometric_centerX", "FLOAT", str(object.Xcenter)])
        properties.append(["has-geometric_centerY", "FLOAT", str(object.Ycenter)])
        properties.append(["has-geometric_centerZ", "FLOAT", str(object.Zcenter)])

        self.client.manipulation.add_batch_dataprop_to_ind(str(object.id), properties)



    def add_coefficients(self, coefficients, object):
        """add coefficients to the individual related to its shape
           check the shape and save in the ontology the relative properties

        @param coefficients(array): array of coefficients returned by pit
        @param object(class Object): individual that represent the old state
        """
        prop = []
        if (object.shape == 'sphere'):
            self.client.manipulation.add_dataprop_to_ind("has-sphere_radius", str(object.id), "FLOAT", str(coefficients[3]))
            object.init_sphere(coefficients[3])
        else:
            if (object.shape == 'cylinder'):
                # create the list for the function
                prop.append(["has-cylinder_radius", "FLOAT", str(coefficients[6])])
                prop.append(["has-cylinder_height", "FLOAT", str(coefficients[7])])
                prop.append(["has-geometric_axisX", "FLOAT", str(coefficients[3])])
                prop.append(["has-geometric_axisY", "FLOAT", str(coefficients[4])])
                prop.append(["has-geometric_axisZ", "FLOAT", str(coefficients[5])])
                prop.append(["has-cylinder_pointX", "FLOAT", str(coefficients[0])])
                prop.append(["has-cylinder_pointY", "FLOAT", str(coefficients[1])])
                prop.append(["has-cylinder_pointZ", "FLOAT", str(coefficients[2])])
                ## save properties
                object.init_cylinder(coefficients[6], coefficients[7], coefficients[3],
                                     coefficients[4], coefficients[5], coefficients[0],
                                     coefficients[1], coefficients[2])

            if (object.shape == 'cone'):
                ## create the list for the function
                prop.append(["has-cone_radius", "FLOAT", str(coefficients[6])])
                prop.append(["has-cone_height", "FLOAT", str(coefficients[7])])
                prop.append(["has-geometric_axisX", "FLOAT", str(coefficients[3])])
                prop.append(["has-geometric_axisY", "FLOAT", str(coefficients[4])])
                prop.append(["has-geometric_axisZ", "FLOAT", str(coefficients[5])])
                prop.append(["has-cone_apexX", "FLOAT", str(coefficients[0])])
                prop.append(["has-cone_apexY", "FLOAT", str(coefficients[1])])
                prop.append(["has-cone_apexZ", "FLOAT", str(coefficients[2])])
                ## save properties
                object.init_cone(coefficients[6], coefficients[7], coefficients[3],
                                 coefficients[4], coefficients[5], coefficients[0],
                                 coefficients[1], coefficients[2])

            if (object.shape == 'plane'):
                ## create the list for the function
                prop.append(["has-geometric_hessian", "FLOAT", str(coefficients[3])])
                prop.append(["has-geometric_axisX", "FLOAT", str(coefficients[0])])
                prop.append(["has-geometric_axisY", "FLOAT", str(coefficients[1])])
                prop.append(["has-geometric_axisZ", "FLOAT", str(coefficients[2])])
                ## save properties
                object.init_plane(coefficients[3], coefficients[0], coefficients[1], coefficients[2])

            self.client.manipulation.add_batch_dataprop_to_ind(str(object.id), prop)


    def check_property(self, ind_id, property, type, new_value, old_value):
        """check if the new property is equal to the old one with a certain threshold and if it isn't replace the value

        @param ind_id(float): number that represent the id of the individual
        @param property (string): name of the property to check
        @param type (string): type of the property
        @param new_value (str/float): it is the latest value returned by pit
        @param old_value (str/float): it is the value contained in the old state

        @return  return the new value in order to update the old state: if the value as been replaced return the new value,
        @return else return the old one
        """
        if isinstance(new_value, str):
            if (new_value != old_value):
                self.client.manipulation.replace_dataprop_b2_ind(property, str(ind_id), type, new_value, old_value)
                return new_value
        else:
            if (abs(new_value - old_value) > self.threshold):
                self.client.manipulation.replace_dataprop_b2_ind(property, str(ind_id), type, str(new_value), str(old_value))
                return new_value

        return old_value

    def check_properties_for_shapes(self, object, coefficients):
        """ for each shape check if the values are different and if it is replace the old value in the ontology
         with the new one and changes also the old state

         @param object(class Object): the individual representing the old state
         @param coefficients (array): array of coefficients given by pit containing all the properties of the shape
        """
        if (object.shape == 'sphere'):
            object.radius = self.check_property(object.id, "has-sphere_radius", "FLOAT", coefficients[3],
                                                object.radius)

        if (object.shape == 'cylinder'):
            object.radius = self.check_property(object.id, "has-cylinder_radius", "FLOAT", coefficients[6],
                                                object.radius)
            object.height = self.check_property(object.id, "has-cylinder_height", "FLOAT", coefficients[7],
                                                object.height)
            object.axisX = self.check_property(object.id, "has-geometric_axisX", "FLOAT", coefficients[3],
                                               object.axisX)
            object.axisY = self.check_property(object.id, "has-geometric_axisY", "FLOAT", coefficients[4],
                                               object.axisY)
            object.axisZ = self.check_property(object.id, "has-geometric_axisZ", "FLOAT", coefficients[5],
                                               object.axisZ)
            object.pointX = self.check_property(object.id, "has-cylinder_pointX", "FLOAT", coefficients[0],
                                                object.pointX)
            object.pointY = self.check_property(object.id, "has-cylinder_pointY", "FLOAT", coefficients[1],
                                                object.pointY)
            object.pointZ = self.check_property(object.id, "has-cylinder_pointZ", "FLOAT", coefficients[2],
                                                object.pointZ)
        if (object.shape == 'cone'):
            object.radius = self.check_property(object.id, "has-cone_radius", "FLOAT", coefficients[6],
                                                object.radius)
            object.height = self.check_property(object.id, "has-cone_height", "FLOAT", coefficients[7],
                                                object.height)
            object.axisX = self.check_property(object.id, "has-geometric_axisX", "FLOAT", coefficients[3],
                                               object.axisX)
            object.axisY = self.check_property(object.id, "has-geometric_axisY", "FLOAT", coefficients[4],
                                               object.axisY)
            object.axisZ = self.check_property(object.id, "has-geometric_axisZ", "FLOAT", coefficients[5],
                                               object.axisZ)
            object.pointX = self.check_property(object.id, "has-cone_apexX", "FLOAT", coefficients[0],
                                                object.pointX)
            object.pointY = self.check_property(object.id, "has-cone_apexY", "FLOAT", coefficients[1],
                                                object.pointY)
            object.pointZ = self.check_property(object.id, "has-cone_apexZ", "FLOAT", coefficients[2],
                                                object.pointZ)
        if (object.shape == 'plane'):
            object.hessian = self.check_property(object.id, "has-geometric_hessian", "FLOAT", coefficients[3],
                                                 object.hessian)
            object.axisX = self.check_property(object.id, "has-geometric_axisX", "FLOAT", coefficients[0],
                                               object.axisX)
            object.axisY = self.check_property(object.id, "has-geometric_axisY", "FLOAT", coefficients[1],
                                               object.axisY)
            object.axisZ = self.check_property(object.id, "has-geometric_axisZ", "FLOAT", coefficients[2],
                                               object.axisZ)


    def remove_coefficients(self, object):

        """remove from the ontology properties that are changed in relation to the shape of the object
         check the shape and save in the ontology the relative properties

        @param objects (class Object): indivial that represent the old state

        """
        prop = []
        if (object.shape == 'sphere'):
            self.client.manipulation.remove_dataprop_from_ind("has-sphere_radius", str(object.id), "FLOAT", str(object.radius))

        else:
            if (object.shape == 'cylinder'):
                ## create the list for the function
                prop.append(["has-cylinder_radius", "FLOAT", str(object.radius)])
                prop.append(["has-cylinder_height", "FLOAT", str(object.height)])
                prop.append(["has-geometric_axisX", "FLOAT", str(object.axisX)])
                prop.append(["has-geometric_axisY", "FLOAT", str(object.axisY)])
                prop.append(["has-geometric_axisZ", "FLOAT", str(object.axisZ)])
                prop.append(["has-cylinder_pointX", "FLOAT", str(object.pointX)])
                prop.append(["has-cylinder_pointY", "FLOAT", str(object.pointY)])
                prop.append(["has-cylinder_pointZ", "FLOAT", str(object.pointZ)])

            if (object.shape == 'cone'):
                ## create the list for the function
                prop.append(["has-cone_radius", "FLOAT", str(object.radius)])
                prop.append(["has-cone_height", "FLOAT", str(object.height)])
                prop.append(["has-geometric_axisX", "FLOAT", str(object.axisX)])
                prop.append(["has-geometric_axisY", "FLOAT", str(object.axisY)])
                prop.append(["has-geometric_axisZ", "FLOAT", str(object.axisZ)])
                prop.append(["has-cone_apexX", "FLOAT", str(object.pointX)])
                prop.append(["has-cone_apexY", "FLOAT", str(object.pointY)])
                prop.append(["has-cone_apexZ", "FLOAT", str(object.pointZ)])

            if (object.shape == 'plane'):
                ## create the list for the function
                prop.append(["has-geometric_hessian", "FLOAT", str(object.hessian)])
                prop.append(["has-geometric_axisX", "FLOAT", str(object.axisX)])
                prop.append(["has-geometric_axisY", "FLOAT", str(object.axisY)])
                prop.append(["has-geometric_axisZ", "FLOAT", str(object.axisZ)])

            self.client.manipulation.remove_batch_dataprop_to_ind(str(object.id), prop)
