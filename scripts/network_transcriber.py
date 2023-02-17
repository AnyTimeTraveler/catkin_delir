import socket

import AbstractSpeechTranscriber
import rospy

class NetworkTranscriber(AbstractSpeechTranscriber.AbstractSpeechTranscriber):
    def __init__(self, address, port):
        self.stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.stream.connect((address, port))

    def recognizeLiveSpeech(self, language="de"):
        pass

    def transcribePartially(self, language="de") -> str:
        while True:
            data = self.stream.recv(1024)
            text = data.decode("utf-8", "ignore")
            rospy.logdebug("Received: " + text)
            if text.startswith("T"):
                rospy.logdebug("Accepted: " + text)
                return text.split(":")[2]

    def transcribe(self, language="de") -> str:
        while True:
            data = self.stream.recv(1024)
            text = data.decode("utf-8", "ignore")
            rospy.logdebug("Received: " + text)
            if text.startswith("F"):
                rospy.logdebug("Accepted: " + text)
                return text.split(":")[2]
