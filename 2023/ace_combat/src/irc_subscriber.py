#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import irc.client

IRC_ADDRESS = "127.0.0.1"  # IRC server address
IRC_PORT = 6667  # IRC server port
IRC_CHANNEL = "#fire"  # IRC channel to subscribe to


class IrcSubscriber:
    def __init__(self, server, channel):
        # Set up IRC client and connect to the server
        self.client = irc.client.Reactor()
        self.server = self.client.server()
        self.server.connect(server, IRC_PORT, "ros-subscriber")

        # Join the specified IRC channel
        self.server.join(channel)

        # Create a ROS publisher for the received messages
        self.pub = rospy.Publisher('irc_messages', String, queue_size=10)
        self.server.add_global_handler("privmsg", self.handle_message)


    def handle_message(self, connection, event):
        if event.type == "privmsg":
            rospy.loginfo("{}: {}".format(event.source.split("!")[0], event.arguments[0]))
            # rospy.loginfo("Received IRC message: {}".format(message.arguments[0]))
            # self.pub.publish(message.arguments[0])

    def listen(self):
        while not rospy.is_shutdown():
            self.client.process_once()
            if self.server.is_connected():
                self.server.ping(IRC_ADDRESS)
            rospy.sleep(0.01)
            

def irc_subscriber():
    rospy.init_node('irc_subscriber', anonymous=True)
    subscriber = IrcSubscriber(IRC_ADDRESS, IRC_CHANNEL)
    subscriber.listen()

if __name__ == '__main__':
    try:
        irc_subscriber()
    except rospy.ROSInterruptException:
        pass