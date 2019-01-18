import numpy as np
from matplotlib import pyplot as plt
import rospy
from test_driver.msg import Combined
from std_msgs.msg import Float64


    

if __name__ == '__main__':
    counter = 0

    rospy.init_node("plotter")
    rospy.Subscriber("position_measurements", Subject, plot_x)
    plt.ion()
    plt.show()
    rospy.spin()



def callback(data):
    global counter
    if counter % 10 == 0:
        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9
        plt.plot(msg.position.y, msg.position.x, '*')
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('graph', anonymous=True)

    rospy.Subscriber("/test_robot/combined_commands", Float64, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()