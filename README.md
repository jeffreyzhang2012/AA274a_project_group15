# asl_turtlebot

This contains a _starting point_ for your final project. Below are _brief_
descriptions of the code. You are strongly encouraged to take a closer look into
the code for more details of how and what the code does.

**File Descriptions:**

**Gazebo Simulation Files:**
----------------------

&#128994; `world/project_city.world`: Defines 3D model of rough, tentative
representation of the final project environment.

**Turtlebot Files:**
----------------------
**Launch Files:**

&#128994; `launch/turtlebot3_bringup_jetson_pi.launch`: Launches the core elements of the
turtlebot stack (turtlebot drivers, camera, lidar, gmapping, static tf
transforms). This should run onboard the jetson.

`launch/turtlebot3_gmapping_sim.launch`:

`launch/turtlebot3_maze.launch`:

`launch/turtlebot3_nav_sim.launch`:

`launch/turtlebot3_signs_sim.launch`:

`launch/turtlebot3_sim.launch`:

`launch/project_sim.launch`:

&#128994; `launch/project_sim.launch`: Launches gazebo with a (rough, tentative)
model of the final project environment, as well as the core SLAM and detector
nodes. You'll need to run your navigator and other project nodes separately.

&#128994; `launch/section4_demo.launch`:

`launch/turtlebot3_arena.launch`:

`launch/turtlebot3_bringup_jetson_pi.launch`:

`launch/turtlebot3_gmapping.launch`:

`launch/turtlebot3_nav.launch`:

`launch/turtlebot3_signs.launch`:

**Scripts/Nodes:**

&#10060; `scripts/camera_relay.py`: Due to networking limitations, your remote
machine cannot access messages directly from the raspberry pi. This node
forwards images from the raspberry pi via the jetson to the remote machine.

&#128994; `scripts/goal_commander.py`: Translates Rviz nav goal clicks
(/move_simple_base/goal) to the /cmd_nav topic.

&#128994; `scripts/detector.py`: Gazebo stop sign detector from HW2. Publishes to
/detector/* where * is the detected object label.

&#128994; `scripts/detector_mobilenet.py`: Runs tensorflow mobilenet model for image
classification. Publishes to /detector/* where * is the detected object label.
**DISCLAIMER:** The distance estimation is not always very accurate and is
noisy. It subscribes to the /scan which takes the closest point (in xy-distance)
from any laserscan ring below the horizontal ring, ignoring all points a
threshold z_min below the velodyne as ground points. For the current
configuration of the Turtlebot, we have set z_min = 16cm. You can combine the
camera and/or point cloud to improve the estimate of the distance.

&#128994; `scripts/detector_viz.py`: Visualizes camera feed, bounding boxes and
confidence for detected objects.

&#10060; `scripts/grids.py`: Used for motion planning. Performs collision checking on
occupancy grids. grids.py functions/classes are used by scripts/navigator.py.

&#128994; `scripts/navigator.py`: Node that manages point to point robot navigation, uses
your A\* implementation (HW2) in an MPC framework along with cubic spline
interpolation and the differential flatness controller (from HW1), switching to
the pose controller from HW1 when close to the goal.

&#128994; `scripts/utils.py`: Utility functions. Currently contains a wrapToPi function,
but feel free to add to this.

`scripts/request_publisher.py`: Utility to submit a delivery request. We'll use
this to send orders for the project.

`scripts/navigator.py`
`scripts/planners`
`scripts/__pycache__`
`scripts/localization.py`
`scripts/map_fixing.py`
`scripts/map_registration.py`
`scripts/pose_controller_nav.py`
`scripts/pose_controller.py`
`scripts/puddle_viz.py`
`scripts/redis_relay.py`
`scripts/request_publisher.py`
`scripts/supervisor_nav.py`
`scripts/supervisor.py`
`scripts/visual_servo.py`
`scripts/camera_relay.py`
`scripts/camera_transform_relay.py`
`scripts/controller.py`
`scripts/controllers`
`scripts/detector_mobilenet.py`
`scripts/detector.py`
`scripts/detector_viz.py`
`scripts/gazebo_plot.py`
`scripts/goal_commander.py`
`scripts/gripper_publisher.py`
`scripts/gripper_relay.py`
`scripts/gripper_sim_controller.py`
`scripts/gripper_subscriber.py`
`scripts/utils.py`
`scripts/grids.py`
`scripts/__init__.py`

**Rviz Configurations:**

***Files From HW***

scripts/controllers/ should contain `P1_pose_stabilization.py` and
`P2_trajectory_tracking.py` from HW1 scripts/planners/ should contain
`P1_astar.py` from HW2


**Message Definitions:**

&#128994; `msg/DetectedObject.msg`: Custom message type that describes detected objects.
Contains the following fields:

uint32 id - Label identifying number

string name - Name of identified object

float64 confidence - Classification probability

float64 distance - Distance to object (**DISCLAIMER:** current implementation
relies on /scan topic for distance (see detector.py and detector_mobilenet.py).
The distance estimation for works in gazebo for stop signs (HW2) but not tested
on hardware)

float64 thetaleft - Left bounding ray of object.

float64 thetaright - Right bounding ray of object.

float64[] corners - Corners of bounding box around detected object with respect
to the tf camera frame.

&#128994; `msg/DetectedObjectList.msg`: Custom message type consisting of a
list/array of DetectedObject objects and their names. Contains the following
fields:

string[] objects - Array of strings corresponding to object names.

DetectedObject[] ob_msgs - Array of DetectedObject objects.


**Tensorflow Models:**

The `.pb` files in the `tfmodels` folder are "frozen" neural network models, and
contain both the structure and the weights of pretrained networks.
`ssd_mobilenet_v1_coco.pb` is a pretrained MobileNet v1 model, while
`stop_sign_gazebo.pb` is a model fine-tuend to detect stop signs in Gazebo. We
recommend using `ssd_resnet_50_fpn.pb`, which is a larger, more accurate and
robust model, but does not fit on a GitHub repo and can be downloaded
[here](https://stanford.app.box.com/s/vszjfhwkjb203qbwhzoirn3uzt5r16lv).

The `coco_labels.txt` file just contains the mapping from the class number
output by the model to human-interpretable labels.

There are many other pretrained models you could use, see the [Tensorflow
detection model
zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md)
for more.


**Other:**

&#128994; `env_pi.sh`: Script to remote launch nodes on the raspberry pi from the jetson.
This overcomes the need to ssh into the raspberry pi separately from the jetson
to launch the camera node. This goes in ~/catkin_ws/devel/ on the raspberry pi.

&#128994; `roslocal.sh`, `rostb3.sh`: Scripts to set your ROS IP settings.

&#128994; `CMakeLists.txt`: CMake file for the package
