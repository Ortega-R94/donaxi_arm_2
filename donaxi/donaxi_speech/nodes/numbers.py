#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import sys

class TalkBack:
    def __init__(self, script_path):
        rospy.init_node('talkback')

        rospy.on_shutdown(self.cleanup)
        self.voice = rospy.get_param("~voice", "voice_don_diphone")
        self.wavepath = rospy.get_param("~wavepath", script_path + "/../sounds")
        self.soundhandle = SoundClient()
        rospy.sleep(1)
        self.soundhandle.stopAll()
        self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        rospy.sleep(1)
        self.soundhandle.say("Ready", self.voice)
        rospy.loginfo("Say one of the navigation commands...")
        rospy.Subscriber('/recognizer/output', String, self.talkback)
        
    def talkback(self, msg):
	palabra = msg.data
        rospy.loginfo(palabra)
	if (palabra == "cancel"):
		rospy.loginfo("The number is one")
        self.soundhandle.say(msg.data, self.voice)

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down talkback node...")

if __name__=="__main__":
    try:
        TalkBack(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Talkback node terminated.")
