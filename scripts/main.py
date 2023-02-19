#!/usr/bin/env python3

import difflib
import json
import math
import random
import re
import signal
import string
import sys
import rospy
from std_msgs.msg import String


def signalHandler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


import AbstractSpeechTranscriber as AbstractSpeechTranscriber
import pocket_sphinx_transcriber as pst
import vosk_transcriber as vsk
import network_transcriber as nst
import spacy_download


def spellingBee(transcriber: AbstractSpeechTranscriber.AbstractSpeechTranscriber, specialCharacter: str, publisher,
                requiredErrorCount: int) -> bool:
    """
    Executes the spelling bee test of the CAM ICU. Returns true if more than 2 errors were made, false otherwise.
    :param transcriber:
    :param specialCharacter:
    :param publisher:
    :param requiredErrorCount:
    :return:
    """
    # load content from json file -- TODO: Loading content from the file doesn't work in ROS, so here it is a workaround
    # with open("rainbowwords.json") as json_file:
    # rainbowwords = list(json.load(json_file)["words"])
    # json_file.close()
    rainbowWordJson = '["Casablanca","Ananasbaum"]'

    rainbowWords = list(json.loads(rainbowWordJson))

    rospy.loginfo(f"Es stehen folgende Wörter für den Buchstabiertest zur Auswahl:")
    for number in range(len(rainbowWords)):
        rospy.loginfo(f"{str(number)}: {rainbowWords[number]}")
    number = -1
    while number < 0 or number > (len(rainbowWords) - 1):
        number: int = int(input("Bitte wähle ein Wort mit seiner entsprechenden Nummer aus: "))

    # build string
    builtword: str = ""
    for letter in str(rainbowWords[number]).upper():
        builtword += letter
        builtword += "-"
    builtword = builtword[:-1]
    rospy.loginfo(f"Du hast das Wort {builtword} ausgewählt.")

    rospy.loginfo(f"Buchstaben Erkennung erfolgt mit dem Buchstaben \"{specialCharacter}\" ...")

    if requiredErrorCount > len(rainbowWords[number]):
        requiredErrorCount = len(rainbowWords[number])
    elif requiredErrorCount < 0:
        requiredErrorCount = 0

    errorAnswers: list[int] = []
    for i in range(requiredErrorCount):
        error = -1
        while error < 0 or error in errorAnswers:
            error = math.floor(random.random() * len(rainbowWords[number]))
            errorAnswers += [error]
    errorAnswers.sort()

    # get list of error letters in rainborw word that correspond to the error awnsers
    errorLetters = []
    for i in range(len(errorAnswers)):
        errorLetters += [rainbowWords[number][errorAnswers[i]]]

    rospy.loginfo(f"Es werden folgende Buchstaben manipuliert: {errorLetters}")

    for i in range(len(rainbowWords[number])):
        recognizedWord = transcriber.transcribePartially().upper()
        isActionManipulated = True if i in errorAnswers else False
        isCharacterRecognized = True if recognizedWord.upper()[0] == specialCharacter.upper() else False

        if isCharacterRecognized:
            rospy.loginfo(f"Der Buchstabe \"{specialCharacter}\" wurde erkannt!")
            if not isActionManipulated:
                rospy.loginfo(f"Sende Handgriff")
                publisher.publish("zu")
            else:
                rospy.loginfo(f"Die folgende Aktion wird manipuliert!")
        else:
            rospy.loginfo(f"Der Buchstabe \"{specialCharacter}\" wurde nicht erkannt!")
            if isActionManipulated:
                rospy.loginfo(f"Die folgende Aktion wird manipuliert!")
                publisher.publish("zu")
            else:
                rospy.loginfo(f"Sende keinen Handgriff")

    rospy.loginfo("Buchstabiertest abgeschlossen!")
    if requiredErrorCount > 2:
        rospy.loginfo(f"{requiredErrorCount} Fehler aufgetreten. Delir möglich!")
        return True
    rospy.loginfo(f"{requiredErrorCount} Fehler aufgetreten. Kein Delir!")
    return False


def logicQuestions(transcriber: AbstractSpeechTranscriber.AbstractSpeechTranscriber, publisher,
                   requiredErrorCount: int, numberOfQuestions: int) -> bool:
    """
    Executes the logic questions test of the CAM ICU. Returns true if more than 1 error was made, false otherwise.
    :param transcriber:
    :param publisher:
    :param requiredErrorCount:
    :param numberOfQuestions:
    :return:
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
    # questions = {}
    # with open("ressources/logicquestiontables/questions.json") as json_file:
    #   questions = json.load(json_file)
    #   json_file.close()
    questionsJson = '{"q1":{"question":"Schwimmt ein Stein auf dem Wasser?","answer":"n"},"q2":{"question":"Gibt es Fische im Meer?","answer":"y"},"q3":{"question":"Wiegt ein Kilo mehr als zwei Kilo?","answer":"n"},"q4":{"question":"Kann man mit einem Hammer einen Nagel in die Wand schlagen?","answer":"y"},"q5":{"question":"Können Enten schwimmen?","answer":"y"},"q6":{"question":"Gibt es Elefanten im Meer?", "answer":"n"},"q7":{"question":"Kann man mit einem Hammer Holz sägen?", "answer":"n"},"q8":{"question":"Kann man in einem Bett schlafen?","answer":"y"} }'

    questions = json.loads(questionsJson)
    rospy.loginfo("Es folgen nun die Logikfragen:")

    if requiredErrorCount > numberOfQuestions:
        requiredErrorCount = numberOfQuestions
    elif requiredErrorCount < 0:
        requiredErrorCount = 0

    errorAnswers: list[int] = []
    for i in range(requiredErrorCount):
        error = -1
        while error < 0 or error in errorAnswers:
            error = math.floor(random.random() * numberOfQuestions)
            errorAnswers += [error]
    errorAnswers.sort()

    for i in range(numberOfQuestions):
        rospy.loginfo(f"Logikfrage {i + 1} von {numberOfQuestions}")
        isActionManipulated = True if i in errorAnswers else False

        # initialText = "Kann Holz mit einem Hammer gesägt werden?"
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
            questionMiddledConfidences[question] = sum(questionConfidences[question]) / len(
                questionConfidences[question])
        matchingQuestion = max(questionMiddledConfidences, key=questionMiddledConfidences.get)
        rospy.loginfo(
            f"Antwort auf Frage \"{initialText}\" ist mit einer Konfidenz von {questionMiddledConfidences[matchingQuestion]}:\n\"{questions[matchingQuestion]['question']}\" -> {questions[matchingQuestion]['answer']}")

        if questions[matchingQuestion]['answer'] == 'y':
            rospy.loginfo("Antwort ist JA")
            if not isActionManipulated:
                rospy.loginfo(f"Sende Handgriff")
                publisher.publish("zu")
            else:
                rospy.loginfo(f"Die folgende Aktion wird manipuliert!")
        else:
            rospy.loginfo(f"Antwort ist NEIN")
            if isActionManipulated:
                rospy.loginfo(f"Die folgende Aktion wird manipuliert!")
                publisher.publish("zu")
            else:
                rospy.loginfo(f"Sende keinen Handgriff")

    if requiredErrorCount > 1:
        rospy.loginfo(f"CAM ICU Test mit {requiredErrorCount} Fehlern durchgefallen. Delir ist vorliegend!")
        return True
    rospy.loginfo(f"CAM ICU Test mit {requiredErrorCount} Fehlern bestanden. Kein Delir vorliegend!")
    return False


def waitfor(transcriber, keyword):
    while keyword.lower() not in transcriber.transcribe().lower():
        pass


def main():
    signal.signal(signal.SIGINT, signalHandler)
    publisher = rospy.Publisher('/Movement_String', String, queue_size=1)
    rospy.init_node('deliriumRecognition')
    # transcriber = vsk.VoskTranscriber()
    transcriber = nst.NetworkTranscriber("10.0.0.1", 4444)


    requiredErrorQuoteForSpellingBee = -1
    while requiredErrorQuoteForSpellingBee < 0:
        requiredErrorQuoteForSpellingBee = input(
            "Bitte eine gewünschte Anzahl an Fehlern für den Buchstabierteil eingeben: ")


    numberOfLogicQuestions = -1
    while numberOfLogicQuestions < 0:
        numberOfLogicQuestions = input("Bitte eine gewünschte Anzahl an Logikfragen eingeben: ")
    requiredErrorQuoteForLogicQuestions = -1
    while requiredErrorQuoteForLogicQuestions < 0 or requiredErrorQuoteForLogicQuestions > numberOfLogicQuestions:
        requiredErrorQuoteForLogicQuestions = input(
            "Bitte eine gewünschte Anzahl an Fehlern für den Logikfragenteil eingeben: ")

    spellingBee(transcriber=transcriber, specialCharacter="a", publisher=publisher,
                requiredErrorCount=requiredErrorQuoteForSpellingBee)
    logicQuestions(transcriber=transcriber, publisher=publisher, requiredErrorCount=requiredErrorQuoteForLogicQuestions,
                   numberOfQuestions=numberOfLogicQuestions)


if __name__ == '__main__':
    main()
