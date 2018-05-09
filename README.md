# object_recognition
ROS object recognition using Pitt + Tensorflow
============

This ROS project realizes the recognition and tracking of multiple objects using
a Kinect, the ontology is then saved using ARMOR.
Using **PITT** and **TENSORFLOW** this architecture easily recognize objects providing:

  * Name of the object
  * Position
  * Tracking
  * Shape
  * Geometrical measures

The architecture
------------
![demo1](https://github.com/guelfis/object_recognition/blob/master/img/architecture.jpg)

The architecture is modular and easy to access.
It is divided into three main structures:

**Pitt:** processes the 3D pointcloud and returns position, shape and properties of the objects.
 <!---
|Node  | Description |
| ------| -----------|
| *Object_segmentation* | recognizes the position of the objects |
| *Geometric_tracker*   |  keeps tracking of objects |
| *Ransac_segmentation* | defines the shapes of the objects |
-->

**Rostensorflow:** processes 2D images and returns position and label of the objects


| Node | Description | Publish | Subscribe |
| ------| -----------| ---- | ---- |
| *Broadcaster*   | Provides the transformation matrix of the plane on which the objects lie |  |  |
| *Tensorflow* | Recognizes name and position of the objects | /TensorOutput | /cameraB/rgb/image_rect_color |


These two principal structures are merged together in order to have a single architecture.
This branch merge informations from the two software and save data on the ontology.

| Node | Description | Publish | Subscribe |
| ------| -----------| ---- | ---- |
| *Merger*    |  Compares data coming from Tensorflow and Pitt, merging the objects informations if there is a match between the coordinates provided by the two. | /results | /geometric_tracker/trackedCluster <br> <br> /TensorOutput |
| *Results*    | Save in the ontology the messages received from the merger, adding the shape informations given from Pitt |  | /ransac_segmentation/trackedShapes <br><br> /results |

How to run
------------
* Install  [**TENSORFLOW**](https://pythonprogramming.net/introduction-use-tensorflow-object-detection-api-tutorial/) following the tutorial
* Install [**ROS**](http://wiki.ros.org)
* Install [**cv bridge**](http://wiki.ros.org/cv_bridge)
* Download [**pitt_object_table_segmentation**](https://github.com/EmaroLab/pitt_object_table_segmentation)
* Download [**pitt_geometric_tracking**](https://github.com/EmaroLab/pitt_geometric_tracking)
* Download [**pitt_msgs**](https://github.com/EmaroLab/pitt_msgs)
* Download [**AMOR**](https://github.com/EmaroLab/multi_ontology_reference)
* Download [**ARMOR**](https://github.com/EmaroLab/armor)
* Download [**armor_msgs**](https://github.com/EmaroLab/armor_msgs)
* Install [**rosjava**](http://wiki.ros.org/rosjava)
* Download [**Python client for Armor**](https://github.com/EmaroLab/armor_py_api)

It is necessary to copy the folder object_detection contained in Tensorflow in the folder /tensorflow_node/src.
All the downloaded components have to be in the same folder in which will be saved the object_recognition folder.

Launch the architecture using the single launch files:
* $ roslaunch launch/launch_tensor.launch
* $ roslaunch launch/table_segmentation.launch

or use the launcher for the entire project:
* $ roslaunch launch/launch_project.launch

Limits
------------
* The camera is supposed to be still
* Tensorflow recognizes just a small class of objects, if it can't assign a label no objects are recognized
* Pit recognizes only primitive shapes, while Tensorflow only common objects
* Tensorflow does a two-dimensional analysis of the scene while Pit analyzes a 3D pointcloud
* Errors in recognition are due to pit, the positioning of the frame and the computation of tensor coordinates
* Tensorflow has no tracker, for this reason we use the Pit tracker.

Documentation
------------
* Report
* Doxygen 
<br>
To run doxygen documentation: open the file 'index.htm' in the folder documentation/html with a browser
