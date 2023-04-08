#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import irc.client

#TODO: add keep-alive function (ping)

IRC_ADDRESS = "127.0.0.1"  # IRC server address
IRC_PORT = 6667  # IRC server port
IRC_CHANNEL = "#fire"  # IRC channel to subscribe to

class IrcSubscriber(irc.client.SimpleIRCClient):
    def __init__(self):
        irc.client.SimpleIRCClient.__init__(self)
        rospy.Timer(rospy.Duration(30), self.keep_alive)
        self.pub = rospy.Publisher('irc_messages', String, queue_size=10)

    def on_welcome(self, connection, event):
        connection.join(IRC_CHANNEL)
        
    def on_pubmsg(self, connection, event):
        if event.target == IRC_CHANNEL:
            msg_str = '{}: {}'.format(event.source.nick, event.arguments[0])
            rospy.loginfo(msg_str)
            self.pub.publish(msg_str)

    def keep_alive(self, event):
        self.connection.ping(IRC_ADDRESS)

if __name__ == '__main__':
    rospy.init_node('irc_subscriber')
    client = IrcSubscriber()
    client.connect(IRC_ADDRESS, IRC_PORT, "ros-subscriber-bot")
    client.start()