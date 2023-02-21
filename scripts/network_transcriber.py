import socket

import AbstractSpeechTranscriber

from scripts.LogHandler import LogHandler
import scripts.LOGLEVEL as LOGLEVEL


class NetworkTranscriber(AbstractSpeechTranscriber.AbstractSpeechTranscriber):
    def __init__(self, address:str, port:int, logger: LogHandler):
        self.pathForAudioFiles = None
        self.stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.stream.connect((address, port))
        self.logger = logger

    def recognizeLiveSpeech(self, language="de"):
        pass

    def transcribePartially(self, language="de") -> str:
        while True:
            data = self.stream.recv(1024)
            text = data.decode("utf-8", "ignore")
            self.logger.log("Received: " + text, LOGLEVEL.DEBUG)
            if text.startswith("T"):
                self.logger.log("Accepted: " + text, LOGLEVEL.DEBUG)
                return text.split(":")[2]

    def transcribe(self, language="de") -> str:
        while True:
            data = self.stream.recv(1024)
            text = data.decode("utf-8", "ignore")
            self.logger.log("Received: " + text, LOGLEVEL.DEBUG)
            if text.startswith("F"):
                self.logger.log("Accepted: " + text, LOGLEVEL.DEBUG)
                return text.split(":")[2]

    def setAudioFile(self, pathForAudioFiles) -> None:
        self.pathForAudioFiles = pathForAudioFiles
