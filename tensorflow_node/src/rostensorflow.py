#!/usr/bin/env python

"""
@ file rostensorflow.py

@ author Guelfi_Serena,Garello_Luca

@ Is the ros node for Tensorflow, the software for object recognition.
@ It detects objects from kinect and returns coordinates and label for each object.
@ The node wait for amor execution before start and works as a listener for the transformation
@ between 'world' frame and 'tensor' frame. It receives images from the kinect and elaborates
@ them returning label,confidence and position of the detected objects.
@ Returned Coordinates are computed as the center of the bounding box  and are converted from pixel 
@ to meters and are multiplied with the transformation matrix T in order to change their referring frame. 
"""


import rospy
import os
import datetime
from sensor_msgs.msg import Image
from tf_msgs.msg import TensorOutput
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tfw
import sys
import tarfile
import tf
import tf2_ros
import collections
import functools
#import PIL.Image as Image
import PIL.ImageColor as ImageColor
import PIL.ImageDraw as ImageDraw
import PIL.ImageFont as ImageFont
import six
from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from tf import transformations
#sys.path.append("./object_detection")
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

#transformation matrix between the frame 'WORLD' and the frame 'TENSOR'
Tmatrix = []

class RosTensorFlow():

    ## get from the launcher the topic of the kinect to use for the detection
    camera_topic = rospy.get_param('/input_camera')
    ##  get from the launch file the factor for the conversion from pixel to meters;
    #it is the ratio between the real dimension in meters of the image and its resolution
    conversion_factor=rospy.get_param('/coordinate_conversion_factor')

    def __init__(self):

        self._cv_bridge = CvBridge()
        ##subscribe to kinect rgb camera
        self._sub = rospy.Subscriber(self.camera_topic, Image, self.callback, queue_size=1)
        ##publish to the merger node on the TensorOutput topic
        self._pub = rospy.Publisher('/TensorOutput', TensorOutput, queue_size=3)


    def callback(self, image_msg):
        """The function receives informations from the kinect and elaborate them in order to detect objects.
            It takes 2D coordinates and label for each object and write them on 'TensorOutput' message and published it on
            /TensorOutput topic"""

        #convert ROS image message to openCV image
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

        with detection_graph.as_default():
            with tfw.Session(graph=detection_graph) as sess:
                image_np = cv_image
                ## Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image_np, axis=0)
                image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                ## Each box represents a part of the image where a particular object was detected.
                boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
                ## Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                scores = detection_graph.get_tensor_by_name('detection_scores:0')
                classes = detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = detection_graph.get_tensor_by_name('num_detections:0')

                # Actual detection.
                (boxes, scores, classes, num_detections) = sess.run(
                    [boxes, scores, classes, num_detections],
                    feed_dict={image_tensor: image_np_expanded})

                # Result of the detection
                obj, X, Y = self.detection(
                            image_np,
                            np.squeeze(boxes),
                            np.squeeze(classes).astype(np.int32),
                            np.squeeze(scores),
                            category_index)
                Xn=[]
                Yn=[]
                print "detected",len(X),"objects"

                #Transforms coordinates from the frame 'tensor' to the frame 'world'
                # by multiplying coordinates with the transformation matrix
                for i in range(0, len(X)):
                    v = np.array([[X[i]], [Y[i]], [0], [1]])
                    w = Tmatrix.dot(v)
                    Xn.append(w[0])
                    Yn.append(w[1])

                #write data on message
                msgout = TensorOutput()
                msgout.header.stamp.secs = datetime.datetime.now().second
                msgout.header.stamp.nsecs = (datetime.datetime.now().microsecond * 1000)
                msgout.result=obj
                msgout.x = Xn
                msgout.y = Yn

                ##publish the message
                self._pub.publish(msgout)

    def detection(self,image,
                  boxes,
                  classes,
                  scores,
                  category_index,
                  instance_masks=None,
                  keypoints=None,
                  max_boxes_to_draw=20,
                  min_score_thresh=.5,
                  agnostic_mode=False):

        """LABELS THE IMAGE AND RETURN THE X,Y COORDINATES
        Args:
          image: uint8 numpy array with shape (img_height, img_width, 3)
          boxes: a numpy array of shape [N, 4]
          classes: a numpy array of shape [N]. Note that class indices are 1-based,
            and match the keys in the label map.
          scores: a numpy array of shape [N] or None.  If scores=None, then
            this function assumes that the boxes to be plotted are groundtruth
            boxes and plot all boxes as black with no classes or scores.
          category_index: a dict containing category dictionaries (each holding
            category index `id` and category name `name`) keyed by category indices.
          instance_masks: a numpy array of shape [N, image_height, image_width], can
            be None
          keypoints: a numpy array of shape [N, num_keypoints, 2], can be None
          max_boxes_to_draw: maximum number of boxes to visualize.  If None, draw all boxes.
          min_score_thresh: minimum score threshold for a box to be visualized
          agnostic_mode: boolean (default: False) controlling whether to evaluate in
            class-agnostic mode or not.  This mode will display scores but ignore classes.

        Returns: LABEL + COORDINATES
        """
        obj = []
        X = []
        Y = []

        box_to_display_str_map = collections.defaultdict(list)
        box_to_color_map = collections.defaultdict(str)
        box_to_instance_masks_map = {}
        box_to_keypoints_map = collections.defaultdict(list)
        if not max_boxes_to_draw:
            max_boxes_to_draw = boxes.shape[0]
        for i in range(min(max_boxes_to_draw, boxes.shape[0])):
            if scores is None or scores[i] > min_score_thresh:
                box = tuple(boxes[i].tolist())
                if instance_masks is not None:
                    box_to_instance_masks_map[box] = instance_masks[i]
                if keypoints is not None:
                    box_to_keypoints_map[box].extend(keypoints[i])
                if scores is None:
                    box_to_color_map[box] = 'black'
                else:
                    if not agnostic_mode:
                        if classes[i] in category_index.keys():
                            class_name = category_index[classes[i]]['name']
                        else:
                            class_name = 'N/A'
                        display_str = '{}: {}%'.format(
                            class_name,
                            int(100 * scores[i]))
                        obj.append(display_str)
                    else:
                        display_str = 'score: {}%'.format(int(100 * scores[i]))

                    box_to_display_str_map[box].append(display_str)
                    if agnostic_mode:
                        box_to_color_map[box] = 'DarkOrange'
                    else:
                        box_to_color_map[box] = vis_util.STANDARD_COLORS[
                            classes[i] % len(vis_util.STANDARD_COLORS)]
        # Calculate the center and return the coordinates converted in meters
        for box, color in box_to_color_map.items():
            ymin, xmin, ymax, xmax = box
            X.append(((xmin + xmax) * 640 / 2) * self.conversion_factor)
            Y.append(((ymin + ymax) * 480 / 2) * self.conversion_factor)
        return obj, X, Y


    def main(self):

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('rostensorflow')

    dir =os.path.dirname(os.path.abspath(__file__))

    MODEL_NAME = dir + '/dataset/ssd_mobilenet_v1_coco_11_06_2017'
    MODEL_FILE = MODEL_NAME + '.tar.gz'
    # Path to frozen detection graph. This is the actual model that is used for the object detection.
    PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'
    # List of the strings that is used to add correct label for each box.
    PATH_TO_LABELS = os.path.join(dir + '/object_detection/data', 'mscoco_label_map.pbtxt')

    #number of all the known labels that can be assigned to the object
    NUM_CLASSES = 90

    detection_graph = tfw.Graph()
    with detection_graph.as_default():
      od_graph_def = tfw.GraphDef()
      with tfw.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tfw.import_graph_def(od_graph_def, name='')

    # Loading label map
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)


    ## listener to the broadcaster node for the transformation between the frame world and the frame used by tensorflow
    #positioned in the top left corner of the image

    listener = tf.TransformListener()
    try:
        listener.waitForTransform('/world', '/tensor_tf', rospy.Time(), rospy.Duration(4.0))
        (translation, rotation) = listener.lookupTransform('/world', '/tensor_tf', rospy.Time(0))
    except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "exception"

    # create the transformation matrix
    Tmatrix = np.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))


    tensor = RosTensorFlow()
    tensor.main()
