#!/usr/bin/env python

"""
@file objects.py
@author Guelfi_Serena,Garello_Luca

@ Class object keep in memory the old state of the detected objects. It contains all the possible
@ properties that an object can have.
"""

import rospy
import os
from os.path import dirname, realpath
import numpy as np
import sys

class Object():

    def  __init__(self):
        """ Creates an empty object when a new one is recognized """
        self.id = 0
        self.label = None
        self.shape = None
        self.Xcenter = 0
        self.Ycenter = 0
        self.Zcenter = 0
        self.radius = 0
        self.height = 0
        self.hessian = 0
        self.axisX = 0
        self.axisY = 0
        self.axisZ = 0
        self.pointX = 0  #apexX for cone
        self.pointY = 0  #apexY
        self.pointZ = 0  #apexZ
        self.color = None

    def init_common_value(self,ID,LABEL,SHAPE,X,Y,Z):
        """ Initializes the object with the common values for all the shapes """
        self.id = ID
        self.label = LABEL
        self.shape = SHAPE
        self.Xcenter = X
        self.Ycenter = Y
        self.Zcenter = Z

    def init_sphere(self,RADIUS):
        """ Initializes the object adding the properties of a sphere  """
        self.radius = RADIUS

    def init_cylinder(self,RADIUS,HEIGHT,AX,AY,AZ,PX,PY,PZ):
        """ Initializes the object adding the properties of a cylinder  """
        self.radius = RADIUS
        self.height = HEIGHT
        self.axisX = AX
        self.axisY = AY
        self.axisZ = AZ
        self.pointX = PX
        self.pointY = PY
        self.pointZ = PZ

    def init_cone(self,RADIUS,HEIGHT,AX,AY,AZ,PX,PY,PZ):
        """ Initializes the object adding the properties of a cone  """
        self.radius = RADIUS
        self.height = HEIGHT
        self.axisX = AX
        self.axisY = AY
        self.axisZ = AZ
        self.pointX = PX
        self.pointY = PY
        self.pointZ = PZ

    def init_plane(self,HESSIAN,AX,AY,AZ):
        """ Initializes the object adding the properties of a plane  """
        self.hessian = HESSIAN
        self.axisX = AX
        self.axisY = AY
        self.axisZ = AZ

    def delete_shapes_coefficients(self):
        """ When an object changes the shape all the old properties are deleted  """
        self.radius = 0
        self.height = 0
        self.hessian = 0
        self.axisX = 0
        self.axisY = 0
        self.axisZ = 0
        self.pointX = 0  #apexX for cone
        self.pointY = 0  #apexY
        self.pointZ = 0  #apexZ
