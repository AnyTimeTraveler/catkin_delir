import numpy as np
import speech_recognition as sr
import AbstractSpeechTranscriber
import LogHandler

import scripts.LOGLEVEL as LOGLEVEL


class PocketSphinxTranscriber(AbstractSpeechTranscriber.AbstractSpeechTranscriber):

    def __init__(self, logger: LogHandler, isMicrophoneUsed: bool = True, model: str = "de-DE"):

        self.pathForAudioFiles = None
        self.logger = logger
        self.isMicrophoneUsed = isMicrophoneUsed
        self.model = model

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

    def encodeText(self):
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
                    audio_data: sr.AudioData = r.record(source)

                    # normalize audio data
                    raw_audio = audio_data.get_raw_data()
                    audio_array = np.frombuffer(raw_audio, dtype=np.int16) / np.iinfo(np.int16).max

                    # generate noise
                    duration = len(audio_array) / audio_data.sample_rate
                    freq = 50
                    time = np.arange(0, duration, 1 / audio_data.sample_rate)
                    amplitude = 0.05
                    #noise = np.random.normal(0, amplitude, len(audio_array))
                    noise = np.sin(2 * np.pi * freq * time) * amplitude
                    combined_audio = audio_array + noise

                    # scale audio data
                    combined_audio *= np.iinfo(np.int16).max

                    # convert to bytes
                    combined_raw_data = combined_audio.astype(np.int16).tobytes()
                    audio = sr.AudioData(combined_raw_data, sample_rate=audio_data.sample_rate,
                                                       sample_width=audio_data.sample_width)

            else:
                self.logger.log("Audiodateinverwendung gefordert, jedoch wurde kein Pfad für eine Audiodatei angegeben.", level=LOGLEVEL.ERROR)
                return None

        # use cmu sphinx to analyze audio
        try:
            text = r.recognize_sphinx(audio, language=self.model)
            self.logger.log(f"Du hast gesagt: {text}", level=LOGLEVEL.DEBUG)
            return text
        except sr.UnknownValueError:
            self.logger.log("Entschuldigung, ich konnte deine Eingabe nicht verstehen.")
        except sr.RequestError as e:
            self.logger.log(f"Entschuldigung, es gab ein Problem bei der Verarbeitung deiner Eingabe: {e}",
                            level=LOGLEVEL.ERROR)
        # sanitize input file
        self.pathForAudioFiles = None

    def transcribePartially(self) -> str:
        return self.encodeText()

    def transcribe(self) -> str:
        return self.encodeText()

    def setAudioFile(self, pathForAudioFiles) -> None:
        self.pathForAudioFiles = pathForAudioFiles
