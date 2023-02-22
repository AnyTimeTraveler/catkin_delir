import json
import wave

import numpy as np
import vosk
import pyaudio
import AbstractSpeechTranscriber

from scripts.LogHandler import LogHandler
import scripts.LOGLEVEL as LOGLEVEL


class VoskTranscriber(AbstractSpeechTranscriber.AbstractSpeechTranscriber):
    def __init__(self, logger: LogHandler,language = "de", isMicrophoneUsed: bool = True, noiseMin: float = 0, noiseMax: float = 0):
        self.pathForAudioFiles = None
        self.logger = logger
        self.isMicrophoneUsed = isMicrophoneUsed
        self.model = vosk.Model(model_path="voskModels/vosk-model-small-de-0.15", lang=language)
        self.noiseMin = noiseMin
        self.noiseMax = noiseMax



    def recognizeLiveSpeech(self):
        if not self.isMicrophoneUsed:
            if self.pathForAudioFiles is not None:
                wf = wave.open(self.pathForAudioFiles, "rb")

                if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getcomptype() != "NONE":
                    self.logger.log("Audio file must be WAV format mono PCM.", LOGLEVEL.ERROR)
                    exit(1)

                # read audio

                frames = wf.getnframes()
                rate = wf.getframerate()
                duration = frames / float(rate)
                audio = wf.readframes(wf.getnframes())

                rec = vosk.KaldiRecognizer(self.model, rate)

                # convert audio to numpy array
                audio_data = np.frombuffer(audio, dtype=np.int16)

                # generate noise

                freq = 50
                time = np.linspace(0, duration, len(audio_data))
                amplitude = np.random.uniform(self.noiseMin, self.noiseMax)
                noise = np.sin(2 * np.pi * freq * time) * amplitude

                # combine audio and noise
                combined_data = audio_data + noise

                # convert back to bytes
                combined_audio = combined_data.astype(np.int16).tobytes()

                rec.AcceptWaveform(combined_audio)


                textjson = rec.FinalResult()
                jsonobj = json.loads(textjson)
                text = jsonobj["text"]
                return text

        else:
            rec = vosk.KaldiRecognizer(self.model, 16000)
            p = pyaudio.PyAudio()
            stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=4000)
            stream.start_stream()
            self.logger.log("Sag etwas:", LOGLEVEL.DEBUG)
            while True:
                data = stream.read(4000, exception_on_overflow=False)
                if len(data) == 0:
                    break
                if rec.AcceptWaveform(data):
                    textjson = rec.Result()
                    jsonobj = json.loads(textjson)
                    text = jsonobj["text"]
                    self.logger.log(f"Du hast gesagt: {text}", LOGLEVEL.DEBUG)
                    stream.stop_stream()
                    stream.close()
                    p.terminate()
                    return text

    def transcribePartially(self) -> str:
        return self.recognizeLiveSpeech()

    def transcribe(self) -> str:
        return self.recognizeLiveSpeech()

    def setAudioFile(self, pathForAudioFiles) -> None:
        self.pathForAudioFiles = pathForAudioFiles
