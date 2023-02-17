import json

import vosk
import pyaudio
import AbstractSpeechTranscriber

class VoskTranscriber(AbstractSpeechTranscriber.AbstractSpeechTranscriber):
    def recognizeLiveSpeech(self, language = "de"):
        model = vosk.Model(lang=language)
        rec = vosk.KaldiRecognizer(model, 16000)

        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=4000)
        stream.start_stream()
        print("Sag etwas:")
        while True:
            data = stream.read(4000, exception_on_overflow=False)
            if len(data) == 0:
                break
            if rec.AcceptWaveform(data):
                textjson = rec.Result()
                jsonobj = json.loads(textjson)
                text = jsonobj["text"]
                print(f"Du hast gesagt: {text}")
                stream.stop_stream()
                stream.close()
                p.terminate()
                return text

    def transcribePartially(self,language="de") -> str:
        return self.recognizeLiveSpeech(language)

    def transcribe(self,language="de") -> str:
        return self.recognizeLiveSpeech(language)



