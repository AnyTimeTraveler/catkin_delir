#!/usr/bin/env python3

import difflib
import json
import re
import signal
import string
import sys
import rospy
from std_msgs.msg import String
import os

def signalHandler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

import AbstractSpeechTranscriber as AbstractSpeechTranscriber
import pocket_sphinx_transcriber as pst
import vosk_transcriber as vsk
import network_transcriber as nst
import spacy_download

def spellingBee(transcriber: AbstractSpeechTranscriber.AbstractSpeechTranscriber, specialCharacter: str, publisher):
   
    # load content from json file -- TODO: Loading content from the file doesn't work in ROS, so here it is a workaround
    #with open("rainbowwords.json") as json_file:
        #rainbowwords = list(json.load(json_file)["words"])
        #json_file.close()
    rainbowwordJson = '["Casablanca","Ananasbaum"]'

    rainbowwords = list(json.loads(rainbowwordJson))

    rospy.loginfo(f"Es stehen folgende Wörter für den Buchstabiertest zur Auswahl:")
    for number in range(len(rainbowwords)):
        rospy.loginfo(f"{str(number)}: {rainbowwords[number]}")
    number = -1
    while number < 0 or number > (len(rainbowwords) - 1):
        number: int = int(input("Bitte wähle ein Wort mit seiner entsprechenden Nummer aus: "))

    # build string
    builtword: str = ""
    for letter in str(rainbowwords[number]).upper():
        builtword += letter
        builtword += "-"
    builtword = builtword[:-1]
    rospy.loginfo(f"Du hast das Wort {builtword} ausgewählt.")

    rospy.loginfo(f"Buchstaben Erkunnung erfolgt mit dem Buchstaben \"{specialCharacter}\" ...")

    for i in range(len(rainbowwords[number])):
        recognizedWord = transcriber.transcribePartially().upper()
        if recognizedWord.upper()[0] == specialCharacter.upper():
            rospy.loginfo("SENDE Buchstabe wurde erkannt!")
            publisher.publish("Buchstabe wurde erkannt!")

    rospy.loginfo("Buchstabiertest abgeschlossen!")


def logicQuestions(transcriber: AbstractSpeechTranscriber.AbstractSpeechTranscriber, publisher):
    """
    Waits for the input of one question that can be answered with yes or no. Question has to be out of the "questions.json" file.
    :return: none
    """
    nlp = spacy_download.load_spacy('de_core_news_sm')

    def trimSentence(text: str) -> str:
        # prepare text for keyword search
        text = text.lower()

        # reduceWords to wordstem
        spacyDoc = nlp(text)
        stemmedTokens = [token.lemma_ for token in spacyDoc]
        text = ' '.join(stemmedTokens)

        # remove symbols
        text = re.sub('[%s]' % re.escape(string.punctuation), '', text)
        return text

    def similarityChecker(stringOne: str, stringTwo: str) -> float:
        """
        Calculates the similarity between two strings
        :param stringOne:
        :param stringTwo:
        :return:
        """
        stringOne = trimSentence(stringOne)
        stringTwo = trimSentence(stringTwo)
        return difflib.SequenceMatcher(None, stringOne, stringTwo).ratio()

    # load content from json file -- TODO: Loading content from the file doesn't work in ROS, so here it is a workaround
    #questions = {}
    #with open("ressources/logicquestiontables/questions.json") as json_file:
     #   questions = json.load(json_file)
     #   json_file.close()
    questionsJson = '{"q1":{"question":"Schwimmt ein Stein auf dem Wasser?","answer":"n"},"q2":{"question":"Gibt es Fische im Meer?","answer":"y"},"q3":{"question":"Wiegt ein Kilo mehr als zwei Kilo?","answer":"n"},"q4":{"question":"Kann man mit einem Hammer einen Nagel in die Wand schlagen?","answer":"y"},"q5":{"question":"Können Enten schwimmen?","answer":"y"},"q6":{"question":"Gibt es Elefanten im Meer?", "answer":"n"},"q7":{"question":"Kann man mit einem Hammer Holz sägen?", "answer":"n"},"q8":{"question":"Kann man in einem Bett schlafen?","answer":"y"} }'

    questions = json.loads(questionsJson)
    rospy.loginfo("Es folgen nun die Logikfragen:")

    #initialText = "Kann Holz mit einem Hammer gesägt werden?"
    initialText = transcriber.transcribe()
    recognizedText = trimSentence(initialText)

    questionConfidences: dict[str:list[float]] = {}
    for question in questions:
        if question not in questionConfidences:
            questionConfidences[question] = []
        questionConfidences[question] += [
            similarityChecker(recognizedText, trimSentence(questions[question]["question"]))]

    questionMiddledConfidences = {}
    for question in questionConfidences:
        questionMiddledConfidences[question] = sum(questionConfidences[question]) / len(questionConfidences[question])
    matchingQuestion = max(questionMiddledConfidences, key=questionMiddledConfidences.get)
    rospy.loginfo(
        f"Antwort auf Frage \"{initialText}\" ist mit einer Konfidenz von {questionMiddledConfidences[matchingQuestion]}:\n\"{questions[matchingQuestion]['question']}\" -> {questions[matchingQuestion]['answer']}")

    rospy.loginfo(f"SENDE Antwort: {questions[matchingQuestion]['answer']}")
    publisher.publish(f"Antwort: {questions[matchingQuestion]['answer']}")


def main():
    signal.signal(signal.SIGINT, signalHandler)
    publisher = rospy.Publisher('speech_recognition', String, queue_size=1)
    rospy.init_node('deliriumRecognition')
    #transcriber = vsk.VoskTranscriber()
    transcriber = nst.NetworkTranscriber("10.0.0.1", 4444)
    spellingBee(transcriber=transcriber, specialCharacter="a", publisher=publisher)
    logicQuestions(transcriber=transcriber, publisher=publisher)


if __name__ == '__main__':
    main()

