 --> Download the data file from: https://myipleiria-my.sharepoint.com/personal/hugo_costelha_ipleiria_pt/_layouts/15/guestaccess.aspx?guestaccesstoken=elJzGfl5QgFxDbIjT1vRMsdH6WrWgdLdgQSu%2bKb%2b0vI%3d&docid=2_0e648d6fbaa0a4e5580e97c2cd630f137&rev=1

 --> Use the following command to store usefull data:
  rosbag record -a -O bagname.bag -x "/p3dx/camera_front/image_raw/(.*)"
  
 --> Use the following command to playback data:
 rosbag play bagname.bag

