#!/usr/bin/env python

"""
    
"""

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import sys

class TalkBack:
    def __init__(self, script_path):
        rospy.init_node('talk')

        rospy.on_shutdown(self.cleanup)
        
        # Set the default TTS voice to use
        self.voice = rospy.get_param("~voice", "voice_don_diphone")
        
        # Set the wave file path if used
        self.wavepath = rospy.get_param("~wavepath", script_path + "/../sounds")
        
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        # Wait a moment to let the client connect to the
        # sound_play server
        rospy.sleep(1)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        
        # Announce that we are ready for input
        self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        rospy.sleep(1)
        self.soundhandle.say("Minion is ready", self.voice)
        
        rospy.loginfo("Say one of the navigation commands...")

        # Subscribe to the recognizer output and set the callback function
        #rospy.Subscriber('/recognizer/output', String, self.talkback)
        
    def talkback(self, msg):
        # Print the recognized words on the screen
        rospy.loginfo(msg.data)
        
        # Speak the recognized words in the selected voice
        self.soundhandle.say(msg.data, self.voice)
        
        # Uncomment to play one of the built-in sounds
        #rospy.sleep(2)
        #self.soundhandle.play(5)
        
        # Uncomment to play a wave file
        #rospy.sleep(2)
        #self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down talk node...")

if __name__=="__main__":
    try:
        TalkBack(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Talk node terminated.")
