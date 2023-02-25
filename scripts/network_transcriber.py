import socket

import AbstractSpeechTranscriber
import rospy
import threading


def loop(transcriber, address, port):
    stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    stream.connect((address, port))
    while True:
        data = stream.recv(1024)
        text = data.decode("utf-8", "ignore")
        text = text.strip()
        rospy.logdebug("Received: " + text)
        recognized = text.split(":")[-1]
        if text.startswith("T"):
            transcriber.partial = recognized
        elif text.startswith("F"):
            transcriber.final = recognized


class NetworkTranscriber(AbstractSpeechTranscriber.AbstractSpeechTranscriber):
    def __init__(self, address, port):
        self.partial = None
        self.final = None
        self.running = True
        t = threading.Thread(target=loop, args=(self,address,port,))
        t.setDaemon(True)
        t.start()

    def transcribePartially(self, language="de") -> str:
        self.partial = None
        while self.partial is None:
            pass
        tmp = self.partial
        self.partial = None
        rospy.loginfo("Partial: " + tmp)
        return tmp

    def transcribe(self, language="de") -> str:
        self.final = None
        while self.final is None:
            pass
        tmp = self.final
        self.final = None
        rospy.loginfo("Final: " + tmp)
        return tmp

