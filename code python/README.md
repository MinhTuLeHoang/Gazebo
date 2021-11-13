# How to install it:

1. cd catkin_ws/src/beginner_tutorials/src in your Ubuntu
2. Download the Deep Learning model, which name "model-370.h5"
3. Download the "detection.py"
    - Note: please install tensorflow package if you haven't yet
    - Note: at line 16, you must change to your path lead to your model-370.h5

# How to use it:

- Note: you can run it anywhere u want

1. Run one world in Gazebo (export waffle_pi & roslaunch)
2. Turn on play button (pause button), to start simulation
3. ```rosrun beginner_tutorials detection.py```
