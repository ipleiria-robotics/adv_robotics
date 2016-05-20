 --> Download the bag file from: https://www.dropbox.com/s/u2fxw04qiitv5gv/pick_and_place_box5.tar.bz2?dl=0

 --> Use the following command to store usefull data:
  rosbag record -a -O bagname.bag -x "/p3dx/camera_front/image_raw/(.*)"
  
 --> Use the following command to playback data:
 rosbag play bagname.bag
