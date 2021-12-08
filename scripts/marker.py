import rospy
from visualization_msgs.msg import Marker
from asl_turtlebot.msg import DetectedObjectList



rospy.Subscriber('/detector/objects', DetectedObjectList, detected_callback)
vis_pub = rospy.Publisher('marker_topic', Marker, queue_size=10)

target_list = defaultdict(tuple)
target_list["fire_hydrant"] = (3.336, 0.32)
target_list["bus"] = (0.4, 0.32)
target_list["suv"] = (1.35, 0.283)
target_list["bowl"] = (2.156, 1.728)

detected_set = {}

marker_id = 0
    

def publisher(name):
    
    # rate = rospy.Rate(1)

    # while not rospy.is_shutdown():
    det_pos = target_list[name]
    det_x = det_pos[0]
    det_y = det_pos[1]

    marker = Marker()

    marker.header.frame_id = "base_camera"
    marker.header.stamp = rospy.Time()

    # IMPORTANT: If you're creating multiple markers, 
    #            each need to have a separate marker ID.
    marker.id = marker_id
    marker_id += 1

    marker.type = 2 # sphere

    marker.pose.position.x = det_x
    marker.pose.position.y = det_y
    marker.pose.position.z = 0.

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    marker.color.a = 1.0 # Don't forget to set the alpha!
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    
    vis_pub.publish(marker)
    # print('Published marker!')
    
    # rate.sleep()


def detected_callback(self, msg):
    detected_name = ""
    for o in msg.ob_msgs:
        if o.name == "car" or o.name == "truck":
            detected_name = "suv"
        if o.name == "bus":
            detected_name = "bus"
        if o.name == "fire_hydrant":
            detected_name = "fire_hydrant"
        if o.name == "bowl" or o.name == "sink":
            detected_name = "bowl"
    if detected_name not in detected_set and detected_name in target_list:
        publisher(detected_name)
    
    

def main():
    rospy.init_node('marker_node', anonymous=True)
    
    

if __name__ == '__main__':
    main()