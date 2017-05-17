#! /usr/bin/env python
import geometry_msgs.msg as gms
import rospy as rp


rp.init_node("bridge")
pub = rp.Publisher("pose", gms.Pose, queue_size=10)

def callback(msg):
    pub.publish(msg.pose)

rp.Subscriber("pose_stamped", gms.PoseStamped, callback=callback)

rp.spin()
