#!/usr/bin/env python

# From Robohub tutorial #3
# http://robohub.org/up-and-flying-with-the-ar-drone-and-ros-handling-feedback/

# Import the standard ROS packages, required for every ROS node.
import roslib
import rospy

# Once the required packages are imported, load the manifest file. 
# This file details the dependencies of the current project (for example, 
# that ardrone_tutorials is dependent on the AR.Drone driver, ardrone_autonomy), 
# which through the following command are loaded into the path.
roslib.load_manifest('ardrone_followme')

# Once the project dependencies have been loaded, we can import those which are needed by the current node. 
# In the following command, we load the Navdata message definition from the ardrone_autonomy package.
from ardrone_autonomy.msg import Navdata

# Here we define a callback function to handle the Navdata messages.
# This function prints a formatted string containing the messages timestamp and the AR.Drones pitch angle.
# This callback function will be used later in the program.
def ReceiveData(data):
    print '[{0:.3f}] Pitch: {1:.3f}'.format(data.header.stamp.to_sec(), data.rotY)

# Before issuing any commands to the rospy package, we need to initialize a ROS node.
rospy.init_node('min_subcriber')

# Next we subscribe the node to the /ardrone/navdata topic, which carries Navdata messages. 
# Received messages should then be handled by the previously-defined ReceiveData function.
sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, ReceiveData)

# Because our program is relying on callbacks for data processing, we need to stop the program from terminating. 
# The above defines a loop that runs until the node is shut down (for example with Ctrl-C). 
# In most situations the above loop would perform some sort of processing, however in our example, 
# we simply pass (do nothing).
while not rospy.is_shutdown():
    pass

