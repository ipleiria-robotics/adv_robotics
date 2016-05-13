 --> Download the bag file from https://www.dropbox.com/s/r4mlxp6sts84wyp/2016-05-13-03-32-28.bag.tar.gz?dl=0

 --> Use the following command to store usefull data:
  rosbag record -a -O bagname.bag -x "/p3dx/camera_front/image_raw/(.*)"
  
 --> Use the following command to playback data:
 rosbag play bagname.bag