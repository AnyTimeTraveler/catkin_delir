import speech_recognition as sr
import AbstractSpeechTranscriber
import os


class PocketSphinxTranscriber(AbstractSpeechTranscriber.AbstractSpeechTranscriber):

    def __init__(self):

        # Create Recognizer-Object with initialized Microphone

        # get mic ID
        # ATTENTION
        # MIC DEVICE ID IS HARDCODED AND DEVICE DEPENDENT
        # mic_id = None

        mic_list = sr.Microphone.list_microphone_names()
       
        for i in range(len(mic_list)):
            rospy.logdebug(f"{i} : {mic_list[i]}")
        self.micId = int(input("Wähle deine Mikrofon ID aus: "))

    def encodeText(self, language="en-US"):
        r = sr.Recognizer()

        r.dynamic_energy_threshold = True
        r.dynamic_energy_adjustment_damping = 0.15
        r.dynamic_energy_adjustment_ratio = 1.5
        r.pause_threshold = 0.8
        r.non_speaking_duration = 0.5
        r.operation_timeout = None
        # start mic and initialize with background noise equalization
        with sr.Microphone(device_index=self.micId) as source:
            r.adjust_for_ambient_noise(source, duration=1)
            rospy.loginfo("Sag etwas:")
            # listen to mic and understand
            audio = r.listen(source)

        # use cmu sphinx to analyze audio
        try:
            text = r.recognize_sphinx(audio, language=language)
            rospy.logdebug(f"Du hast gesagt: {text}")
            return text
        except sr.UnknownValueError:
            rospy.loginfo("Entschuldigung, ich konnte deine Eingabe nicht verstehen.")
        except sr.RequestError as e:
            rospy.logerror(f"Entschuldigung, es gab ein Problem bei der Verarbeitung deiner Eingabe: {e}")

    def transcribePartially(self, language="en-US") -> str:
        return self.encodeText(language)

    def transcribe(self, language="en-US") -> str:
        return self.encodeText(language)
