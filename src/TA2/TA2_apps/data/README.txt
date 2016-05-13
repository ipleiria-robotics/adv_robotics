 --> Download the videos and the bag file from Moodle

 --> Use the following command to store usefull data:
  rosbag record -a -O bagname.bag -x "/p3dx/camera_front/image_raw/(.*)"
  
 --> Use the following command to playback data:
 rosbag play bagname.bag