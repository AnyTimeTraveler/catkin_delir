import socket

import AbstractSpeechTranscriber


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
            print("Received: " + text)
            if text.startswith("T"):
                print("Accepted: " + text)
                return text.split(":")[2]

    def transcribe(self, language="de") -> str:
        while True:
            data = self.stream.recv(1024)
            text = data.decode("utf-8", "ignore")
            print("Received: " + text)
            if text.startswith("F"):
                print("Accepted: " + text)
                return text.split(":")[2]
