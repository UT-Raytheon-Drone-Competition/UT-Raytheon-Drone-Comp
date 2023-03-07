#!/usr/bin/env python

import rospy
from ace_combat.srv import FireAce, FireAceResponse
import irc.client

CHANNEL = "#my-channel"
IRC_MESSAGE = "RTXDC_2023 UT_UAV_Fire_{0}_{1}_{2} {3} {4}"
IRC_ADDRESS = "irc.example.com" # TODO: Change this to a ros parameter

# Set up the IRC client and connect to the server
client = irc.client.IRC()
server = client.server()
server.connect(IRC_ADDRESS, 6667, "my-nickname")

def handle_my_service(req):
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
        server.privmsg(CHANNEL, message)
    except Exception as e:
        rospy.logerr("Failed to send IRC message: {}".format(str(e)))
        return FireAceResponse(status_code=1)
    
    # Return a response indicating success
    return FireAceResponse(status_code=0)

def my_service_server():
    rospy.init_node('ace_fire_server')
    rospy.Service('ace_fire_service', FireAce, handle_my_service)
    rospy.spin()

if __name__ == '__main__':
    my_service_server()
