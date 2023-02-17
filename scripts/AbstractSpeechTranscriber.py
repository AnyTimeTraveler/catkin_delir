from abc import ABC


class AbstractSpeechTranscriber(ABC):

    def transcribePartially(self) -> str:
        """ Returns the first recognized Text"""
        pass

    def transcribe(self) -> str:
        """ Returns only fully recognized Text"""
        pass
