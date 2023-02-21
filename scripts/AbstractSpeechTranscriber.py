from abc import ABC


class AbstractSpeechTranscriber(ABC):

    def transcribePartially(self) -> str:
        """ Returns the first recognized Text"""
        pass

    def transcribe(self) -> str:
        """ Returns only fully recognized Text"""
        pass

    def setAudioFile(self, pathForAudioFiles) -> None:
        """ Sets the path for the audio files"""
        pass
