#!/usr/bin/env python

import rospy
from ace_combat.srv import FireAce, FireAceResponse
import irc.client

IRC_PORT = 6667  # IRC server port
IRC_CHANNEL = "#fire"  # IRC channel to subscribe to
IRC_MESSAGE = "RTXDC_2023 UT_UAV_Fire_{0}_{1}_{2}_{3}"
IRC_ADDRESS = "127.0.0.1" # TODO: Change this to a ros parameter

class IRCPublisher(irc.client.SimpleIRCClient):
    
    def __init__(self):
        irc.client.SimpleIRCClient.__init__(self)
        rospy.init_node('ace_fire_server')
        rospy.Timer(rospy.Duration(30), self.keep_alive)
        self.service = rospy.Service('ace_fire_service', FireAce, self.handle_my_service)
        
    def on_welcome(self, connection, event):
        self.connection = connection
        connection.join("#fire")
        
    def send_message(self, message):
        self.connection.privmsg("#fire", message)

    def keep_alive(self, event):
        self.connection.ping(IRC_ADDRESS)

    def handle_my_service(self, req):
        # TODO: Fire ACE thing
        # Construct the IRC message using the request data
        message = IRC_MESSAGE.format(
            req.aruco_marker_id, 
            req.timestamp, 
            req.latitude, 
            req.longitude
        )
        
        # Send the IRC message to the server
        try:
            rospy.loginfo("Sending IRC message: {}".format(message))
            rospy.loginfo("Sent IRC message to channel {}".format(IRC_CHANNEL))
            self.send_message(message)
        except Exception as e:
            rospy.logerr("Failed to send IRC message: {}".format(str(e)))
            return FireAceResponse(status_code=1)
        
        # Return a response indicating success
        return FireAceResponse(status_code=0)
        
def my_service_server():
    client = IRCPublisher()
    client.connect(IRC_ADDRESS, IRC_PORT, "ros-publisher-bot")
    client.start()
    rospy.spin()

if __name__ == '__main__':
    my_service_server()
