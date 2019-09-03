#!/usr/bin/python
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Header

class DetectionRecorder:
    def __init__(self):
        self.file = open ("test.txt", "w+")
        header = ",".join(["x", "y", "timestamp", "class", "frame_id"])
        self.file.write(header + "\n")
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("/pcl_gpu_tools/detected_objects", PoseArray, self.pose_cb)
        rospy.spin()

    def pose_cb(self,msg):
        stamp = str(msg.header.stamp.to_sec())
        frame_id = msg.header.frame_id
        default_class = "object"

        target_frame = "utm"

        for p in msg.poses:
            transform = self.tf_buffer.lookup_transform(target_frame,
                                           frame_id, #source frame
                                           rospy.Time(0), #get the tf at first available time
                                           rospy.Duration(0.1)) #wait for 1 second

            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = p
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            print(pose_transformed)
            info = ",".join([str(pose_transformed.pose.position.x), str(pose_transformed.pose.position.y), stamp, default_class, target_frame])
            self.file.write(info+"\n")

    def __del__(self):
        print("destructor")
        self.file.close()



if __name__ == '__main__':
    rospy.init_node("detection_recording")
    recorder = DetectionRecorder()
