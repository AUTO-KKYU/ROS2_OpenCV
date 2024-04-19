# ROS2_OpenCV
A simple ROS2 publisher that captures or records a frame from the camera using OpenCV

## Dependencies 
- In order to take a picture, a simple service definition is needed : ***best_camera_msgs***

## Installation
- Please check your camera settings.
- **MAKE SURE YOUR CAMERA HAS BEEN ENABLED.**

Install ROS2 & others
- check my repository : https://github.com/AUTO-KKYU/ROS2-Study

Install OpenCV
```ruby
$ sudo apt update
$ sudo apt upgrade
$ pip3 install opencv-python
$ sudo apt install ros-humble-cv-bridge
```

Clone this repo into the src directory of your ROS2 workspace
```ruby
$ git clone https://github.com/AUTO-KKYU/ROS2_OpenCV.git
```
## prerequisite action
- set your .bashrc file
- You need to update the path for the image below to your own path, and also set the ROS domain.

![image](https://github.com/AUTO-KKYU/ROS2_OpenCV/assets/118419026/94cb6d66-2546-483f-b59f-a3a038bd3ec8)
![Screenshot from 2024-04-19 16-15-03](https://github.com/AUTO-KKYU/ROS2_OpenCV/assets/118419026/ce352447-2231-40af-808e-c931dc530241)




## prepare 3 new terminals 
1. Running the camera_node from the best_camera package
```ruby
$ ros2 run best_camera camera_node 
```

2. To run the camera node, and the service to take a picture and record a video use the provided launch file.
   
   **Note the parameters in the launch file states the resolutions 640x380.**
   
```ruby
$ ros2 launch best_camera camera.launch.py
```

3. Take a picture using this service call
```ruby
$ ros2 service call /service_name service_type "{request_field: value}"
# example
$ ros2 service call /camera/save_picture best_camera_msgs/srv/Save “{name: ‘/home/kkyu/Camera_study/src/best_camera/resource/capture_images/test.jpg}”

```

4. Record a video using this service call
```ruby
# example
$ ros2 service call /camera/record best_camera_msgs/Record "{start: true, file_name: '/home/kkyu/Camera_study/src/best_camera/resource/video_files/example.avi'}"
```

5. Stop Record a video using this service call
```ruby
$ ros2 service call /camera/record best_camera_msgs/srv/Record "{start: false}"
```

## 추후 변경사항
- 경로를 임의로 지정해줘야 불편하지만 일단 OpenCV 캠 노드가 활성화 되고 나서, 해당 노드에 1차적으로 서비스 호출하는 방식을 구현하였음
  - 캡쳐 혹은 비디오 경로에 대한 간편화
- 필터도 같이 적용되는지 확인
