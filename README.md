# How to use this git

1. Go to catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo in your computer
2. Copy exactly the same folder from github to the folder in your computer.


# How to simulate a world with Gazebo
1. ```export TURTLEBOT3_MODEL=waffle_pi``` (only waffle_pi has a camera)
2. ```roslaunch turtlebot3_gazebo <name of world>.launch```
    - There is a list of worlds below that you can choose

# List of world:
1. hill_world.launch:
<img src="./img/hill_world.png" width="240" height="200"/>

2. circle_world.launch 
<img src="./img/circle_world.png" width="240" height="200"/>

3. map_2.launch:

<img src="./img/map2.png" width="320" height="200"/><img src="./img/map_2.world.png" width="240" height="200"/>

4. vat_can_1_world.launch:

> map đơn giản để test phát hiện vật cản và chuyển lane


# List of source code:

1. detection.py:

> Traffic sign detection and classification (bản cuối cùng được Nhân giữ)
> Đi kèm với **model-370.h5**

1. change_lane_Tu.py:
> Khi gõ lệnh sẽ chỉ thực hiện change lane

1. lidar.py:
> khối để gửi message khi phát hiện vật cản

1. lidar_src.py:
> Khối để tôi test khoảng cách giữa các vật