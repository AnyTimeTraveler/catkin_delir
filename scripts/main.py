#!/usr/bin/env python3

import difflib
import json
import logging
import math
import random
import re
import signal
import string
import sys
import AbstractSpeechTranscriber as AbstractSpeechTranscriber
import pocket_sphinx_transcriber as pst
import vosk_transcriber as vsk
import network_transcriber as nst
import spacy_download
import LogHandler

isRosUsed: bool = False


def signalHandler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


def spellingBee(transcriber: AbstractSpeechTranscriber.AbstractSpeechTranscriber, specialCharacter: str, publisher,
                requiredErrorCount: int, logger: LogHandler) -> bool:
    """
    Executes the spelling bee test of the CAM ICU. Returns true if more than 2 errors were made, false otherwise.
    :param logger:
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

    logger.log(f"Es stehen folgende Wörter für den Buchstabiertest zur Auswahl:")
    for number in range(len(rainbowWords)):
        logger.log(f"{str(number)}: {rainbowWords[number]}")
    number = -1
    while number < 0 or number > (len(rainbowWords) - 1):
        number: int = int(input("Bitte wähle ein Wort mit seiner entsprechenden Nummer aus: "))

    # build string
    builtword: str = ""
    for letter in str(rainbowWords[number]).upper():
        builtword += letter
        builtword += "-"
    builtword = builtword[:-1]
    logger.log(f"Du hast das Wort {builtword} ausgewählt.")

    logger.log(f"Buchstaben Erkunnung erfolgt mit dem Buchstaben \"{specialCharacter}\" ...")

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

    logger.log(f"Es werden folgende Buchstaben manipuliert: {errorLetters}")

    for i in range(len(rainbowWords[number])):
        recognizedWord = transcriber.transcribePartially().upper()
        isActionManipulated = True if i in errorAnswers else False
        isCharacterRecognized = True if recognizedWord.upper()[0] == specialCharacter.upper() else False

        if isCharacterRecognized:
            logger.log(f"Der Buchstabe \"{specialCharacter}\" wurde erkannt!")
            if not isActionManipulated:
                logger.log(f"Sende Handgriff")
                publisher.publish("zu")
            else:
                logger.log(f"Die folgende Aktion wird manipuliert!")
        else:
            logger.log(f"Der Buchstabe \"{specialCharacter}\" wurde nicht erkannt!")
            if isActionManipulated:
                logger.log(f"Die folgende Aktion wird manipuliert!")
                publisher.publish("zu")
            else:
                logger.log(f"Sende keinen Handgriff")

    logger.log("Buchstabiertest abgeschlossen!")
    if requiredErrorCount > 2:
        logger.log(f"{requiredErrorCount} Fehler aufgetreten. Delir möglich!")
        return True
    logger.log(f"{requiredErrorCount} Fehler aufgetreten. Kein Delir!")
    return False


def logicQuestions(transcriber: AbstractSpeechTranscriber.AbstractSpeechTranscriber, publisher,
                   requiredErrorCount: int, numberOfQuestions: int, logger: LogHandler) -> bool:
    """
    Executes the logic questions test of the CAM ICU. Returns true if more than 1 error was made, false otherwise.
    :param logger:
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
    logger.log("Es folgen nun die Logikfragen:")

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
        logger.log(f"Logikfrage {i + 1} von {numberOfQuestions}")
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
        logger.log(
            f"Antwort auf Frage \"{initialText}\" ist mit einer Konfidenz von {questionMiddledConfidences[matchingQuestion]}:\n\"{questions[matchingQuestion]['question']}\" -> {questions[matchingQuestion]['answer']}")

        if questions[matchingQuestion]['answer'] == 'y':
            logger.log("Antwort ist JA")
            if not isActionManipulated:
                logger.log(f"Sende Handgriff")
                publisher.publish("zu")
            else:
                logger.log(f"Die folgende Aktion wird manipuliert!")
        else:
            logger.log(f"Antwort ist NEIN")
            if isActionManipulated:
                logger.log(f"Die folgende Aktion wird manipuliert!")
                publisher.publish("zu")
            else:
                logger.log(f"Sende keinen Handgriff")

    if requiredErrorCount > 1:
        logger.log(f"CAM ICU Test mit {requiredErrorCount} Fehlern durchgefallen. Delir ist vorliegend!")
        return True
    logger.log(f"CAM ICU Test mit {requiredErrorCount} Fehlern bestanden. Kein Delir vorliegend!")
    return False


def waitfor(transcriber, keyword):
    while keyword.lower() not in transcriber.transcribe().lower():
        pass


def main():
    signal.signal(signal.SIGINT, signalHandler)  #

    # Setup ROS usage
    rosUsedString: str = ""
    while rosUsedString != "y" and rosUsedString != "n":
        rosUsedString = input("Soll ROS verwendet werden? (y/n): ")

    logger = None
    isRosUsed: bool = rosUsedString == "y"
    if isRosUsed:
        import rospy
        from std_msgs.msg import String
        publisher = rospy.Publisher('/Movement_String', String, queue_size=1)
        rospy.init_node('deliriumRecognition')
        logger = LogHandler.LogHandler(isRosUsed=isRosUsed)
    else:
        import DummyPublisher
        publisher = DummyPublisher.DummyPublisher(topic='/Movement_String')
        logging.basicConfig(level=logging.INFO)
        logger = LogHandler.LogHandler(isRosUsed=isRosUsed)

    # Setup Transcriber
    transcriberWantedNumber: int = -1
    print("Wähle einen zu verwendenden Transcriber aus:\n1: PocketSphinx\n2: Lokal Vosk\n3: Netzwerk Vosk")
    while transcriberWantedNumber != "1" and transcriberWantedNumber != "2" and transcriberWantedNumber != "3":
        transcriberWantedNumber = int(input("Bitte eine Zahl eingeben: "))

    if transcriberWantedNumber == 1:
        transcriber = pst.PocketSphinxTranscriber(logger=logger)
    elif transcriberWantedNumber == 2:
        transcriber = vsk.VoskTranscriber(logger=logger)
    else:
        transcriber = nst.NetworkTranscriber(address="10.0.0.1", port=4444, logger=logger)

    pathForAudioFiles = "../audio/"
    # pathForAudioFiles = ""

    if pathForAudioFiles != "":
        transcriber.setAudioFile(pathForAudioFiles)

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
                requiredErrorCount=requiredErrorQuoteForSpellingBee, logger=logger)
    logicQuestions(transcriber=transcriber, publisher=publisher, requiredErrorCount=requiredErrorQuoteForLogicQuestions,
                   numberOfQuestions=numberOfLogicQuestions, logger=logger)


if __name__ == '__main__':
    main()
