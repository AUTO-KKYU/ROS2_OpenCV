# ROS2_OpenCV
A simple ROS2 publisher that captures or records a frame from the camera using OpenCV

## Dependencies 
- In order to take a picture, a simple service definition is needed : ***best_camera_srv***

## Installation
- Please check your camera settings.
- **MAKE SURE YOUR CAMERA HAS BEEN ENABLED.**

**Install ROS2 & others**
- check my repository : https://github.com/AUTO-KKYU/ROS2-Study

**Install OpenCV**
```ruby
$ sudo apt update
$ sudo apt upgrade
$ pip3 install opencv-python
$ sudo apt install ros-humble-cv-bridge
```

**Clone this repo into the src directory of your ROS2 workspace**
```ruby
$ mkdir -p ~/Best_camera/src
$ cd ~/Best_camera/src
$ git clone https://github.com/AUTO-KKYU/ROS2_OpenCV.git
```
## prerequisite action

**Before proceeding, ensure that you have built the package within your ROS workspace**
```ruby
$ cd ~/Best_camera/src
$ colcon build
$ source install/local_setup.bash
```
---

## prepare 5 new terminals 
1. Running the img_publisher in launch file from the best_camera package
```ruby
$ ros2 launch best_camera camera.launch.py 
```

2. You can apply filters to OpenCV frames
   
```ruby
$ ros2 launch best_camera filter.launch.py 
```

3. If you'd like to capture or record a service, you'll need to run this file
```ruby
$ ros2 launch best_camera picture.launch.py 
```

4. Take a picture using this service call
```ruby
$ ros2 service call /service_name service_type "{request_field: value}"
# example - capture
$ ros2 service call /capture best_camera_srv/srv/Save "{name: 'capture.jpg'}"
```
5. Record a video using this service call
```sh
# example - video Start
$ ros2 service call /record best_camera_srv/srv/Record "{start: true}"
# example - video Stop
$ ros2 service call /record best_camera_srv/srv/Record "{start: false}"
```

## Note
- If you want to change the topic, you should create a service called ChangeTopic.srv.
- It's convenient for capturing or recording from a topic with applied filters.

## Result
![image](https://github.com/AUTO-KKYU/ROS2_OpenCV/assets/118419026/cb2b8506-fc1e-4ad6-9a4e-05b9fe21dacc)

![image](https://github.com/AUTO-KKYU/ROS2_OpenCV/assets/118419026/d6cd55c5-0205-4449-90b4-3b815bba2170)


