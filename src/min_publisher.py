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
roslib.load_manifest('ardrone_tutorials')

# Once the project dependencies have been loaded, we can import those which are needed by the current node. 
# In the following command, we load the Navdata message definition from the ardrone_autonomy package.
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import String

# Save the messages in a queue, rather than process them straight away.
messages = []
def ReceiveData(data):
    messages.append(data)

# Before issuing any commands to the rospy package, we need to initialize a ROS node.
rospy.init_node('min_publisher')

# Next we subscribe the node to the /ardrone/navdata topic, which carries Navdata messages. 
# Received messages should then be handled by the previously-defined ReceiveData function.
sub_Navdata = rospy.Subscriber('/ardrone/navdata', Navdata, ReceiveData)

# Create a new publisher, responsible for publishing messages of the String type to the /ardrone/average_pitch topic.
# Note that we do not need to explicitly initialize this topic, it will be initialized automatically on first usage.
pub_Average = rospy.Publisher('/ardrone/average_pitch', String)

# Define a Rate object. We use this to control the frequency of the main processing loop. 
# In our case, we want to print a message every second, thus we want a frequency of 1.
#
# If messages have been received, we calculate the average pitch using a python list comprehension to extract the 
# pitch (.rotY) from every Navdata message. We then empty the messages queue.
#
# Finally, construct a new String message and set its data before publishing it to the topic we previously created.
r = rospy.Rate(1)
while not rospy.is_shutdown():
    if len(messages)>0:
        avg = sum([m.rotY for m in messages])/len(messages)
        messages = []

        avgmsg = String()
        avgmsg.data = 'Average Pitch: {0:.3f}'.format(avg)
        pub_Average.publish(avgmsg)
    r.sleep()

