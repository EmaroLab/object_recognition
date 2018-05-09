#!/usr/bin/env python


"""
@file broadcaster.py
@author Guelfi_Serena,Garello_Luca
@brief broadcaster node that send to Tensorflow the transform between the frame 'WORLD'
@brief  and the frame 'TENSOR', in order to make the transformaton of the coordinates.
"""

import roslib
roslib.load_manifest('tensorflow_node')
import rospy
import tf



if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        br.sendTransform((0.980, 0.530, 0.000),
                     (0.707, -0.707, 0, 0),
                     rospy.Time.now(),
                     'tensor_tf',
                     'world')

    rospy.spin()
