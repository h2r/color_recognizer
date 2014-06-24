color_recognizer
================

Install ORK from: http://wg-perception.github.io/object_recognition_core/install.html#install

Clone this package within a catkin workspace's source and build using catkin_make:

```
git clone https://github.com/h2r/color_recognizer.git
cd ..
catkin_make
```

To run the package first start the Kinect and run ORK's table top detection:
```
roslaunch openni_launch openni.launch depth_registration:=true
roscd object_recognition_tabletop/
rosrun object_recognition_core  detection -c conf/detection.object.ros.ork 
```


Next run the colour_recognizer:
```
rosrun colour_recognizer colourTopics
```

The object location and colour topics are published in the object_location topic, and can be read using:
```
rostopic echo object_location
```
