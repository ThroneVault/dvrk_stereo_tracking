import rospy
from geometry_msgs.msg import PoseStamped, Pose
import pickle
import os.path
import sys

FILENAME = "./pose_list.p"

pose_list = []

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
    # print "called"
    global pose_list
    
    current_pose = data.pose
    pose_list.append(current_pose)
    pickle.dump(pose_list, open( "pose_list.p", "wb"))
    print "Value appended!\n"
    rospy.signal_shutdown("SHutting down")
    sys.exit()
    
def listener():
    
    global pose_list
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/dvrk/PSM2/position_cartesian_current", PoseStamped, callback, queue_size = 1)

    if os.path.isfile(FILENAME):
        print "file found"
        pose_list = pickle.load(open("pose_list.p", "rb"))
    else:
        print "file not found! Creating new file"
    rospy.spin()
    



if __name__ == '__main__':
    listener()
