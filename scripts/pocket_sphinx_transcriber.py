import speech_recognition as sr
import AbstractSpeechTranscriber
import LogHandler

import scripts.LOGLEVEL as LOGLEVEL


class PocketSphinxTranscriber(AbstractSpeechTranscriber.AbstractSpeechTranscriber):

    def __init__(self, logger: LogHandler, isMicrophoneUsed: bool = True):

        self.pathForAudioFiles = None
        self.logger = logger
        self.isMicrophoneUsed = isMicrophoneUsed

        if self.isMicrophoneUsed:
            # Create Recognizer-Object with initialized Microphone

            # get mic ID
            # ATTENTION
            # MIC DEVICE ID IS HARDCODED AND DEVICE DEPENDENT
            # mic_id = None

            mic_list = sr.Microphone.list_microphone_names()

            for i in range(len(mic_list)):
                self.logger.log(f"{i} : {mic_list[i]}")
            self.micId = int(input("Wähle deine Mikrofon ID aus: "))

    def encodeText(self, language="de-de"):
        r = sr.Recognizer()

        r.dynamic_energy_threshold = True
        r.dynamic_energy_adjustment_damping = 0.15
        r.dynamic_energy_adjustment_ratio = 1.5
        r.pause_threshold = 0.8
        r.non_speaking_duration = 0.5
        r.operation_timeout = None

        if self.isMicrophoneUsed:
            # start mic and initialize with background noise equalization
            with sr.Microphone(device_index=self.micId) as source:
                r.adjust_for_ambient_noise(source, duration=1)
                self.logger.log("Sag etwas:")
                # listen to mic and understand
                audio = r.listen(source)
        else:
            if self.pathForAudioFiles is not None:
                with sr.AudioFile(self.pathForAudioFiles) as source:
                    audio = r.record(source)
            else:
                self.logger.log("Audiodateinverwendung gefordert, jedoch wurde kein Pfad für eine Audiodatei angegeben.", level=LOGLEVEL.ERROR)
                return None

        # use cmu sphinx to analyze audio
        try:
            text = r.recognize_sphinx(audio, language=language)
            self.logger.log(f"Du hast gesagt: {text}", level=LOGLEVEL.DEBUG)
            return text
        except sr.UnknownValueError:
            self.logger.log("Entschuldigung, ich konnte deine Eingabe nicht verstehen.")
        except sr.RequestError as e:
            self.logger.log(f"Entschuldigung, es gab ein Problem bei der Verarbeitung deiner Eingabe: {e}",
                            level=LOGLEVEL.ERROR)
        # sanitize input file
        self.pathForAudioFiles = None

    def transcribePartially(self, language="en-US") -> str:
        return self.encodeText(language)

    def transcribe(self, language="en-US") -> str:
        return self.encodeText(language)

    def setAudioFile(self, pathForAudioFiles) -> None:
        self.pathForAudioFiles = pathForAudioFiles
