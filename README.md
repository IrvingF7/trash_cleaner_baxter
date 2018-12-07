# Trash Cleaner Baxter

## Installation
Use Python 2.7.10

### Clone the Repo
```
git clone https://github.com/GrantMeAWish/trash_cleaner_baxter.git
```

### Setup to run demo
Change bashrc IP
```
gedit ~/.bashrc
```

Navigate to ros workspace and connect to baxter
```
./baxter.sh asimov.local
```

Launch freenect and configure depth registration
```
roslaunch freenect_launch freenect.launch

rosrun rqt_reconfigure rqt_reconfigure

rosrun image_view image_view image:=/camera/rgb/image_color
```

Launch kinect static transform and AR tag tracking. Use rviz to view frames and robot state.
```
roslaunch trash_cleaner_baxter kinect_static_tf.launch

roslaunch ar_track_alvar kinect_ar.launch

rosrun rviz rviz
```