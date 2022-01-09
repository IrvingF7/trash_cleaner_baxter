# Trash Cleaner Baxter
This is Berkeley EECS106A Final Project done by Grant Wang, Tae Kyun Kim and Irving Fang.
A robot using R-CNN to identify different kinds of trashes and cleaning it with appropriate tools.
More details can be found at this [project website](https://sites.google.com/view/trash-cleaner-baxter)

## Installation
Use Python 2.7.10

### Clone the Repo
Clone the repo into your catkin workspace packages. 
```
git clone https://github.com/GrantMeAWish/trash_cleaner_baxter.git
```
Make sure your workspace is sourced approprriately. Then run from workspace
```
catkin_make
```

### Setup to run demo
Change bashrc IP
```
gedit ~/.bashrc
```

Navigate to ros workspace and connect to Baxter for all terminal commands
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

### Running the demo
Once again make sure you are connected to the appropriate Baxter robot with all the setup done. Then run the following script to run the demo.
```
rosrun trash_cleaner_baxter demo.py
```
