#!/usr/bin/env python3

import difflib
import json
import logging
import math
import os
import random
import re
import signal
import string
import sys
import threading

import AbstractSpeechTranscriber as AbstractSpeechTranscriber
import pocket_sphinx_transcriber as pst
import vosk_transcriber as vsk
import network_transcriber as nst
import spacy_download
import LogHandler
import scripts.LOGLEVEL as LOGLEVEL

isRosUsed: bool = False

globalSpellingBeeFileCounter: int = 0
globalLogicQuestionsFileCounter: int = 0

threadingLock = threading.Lock()


def signalHandler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


def getFilePathsOfFilesInSubdirectories(path: str) -> list:
    """
    Returns a list of all files in the given directory and all subdirectories.
    :param path:
    :return:
    """
    filePaths = []
    for root, dirs, files in os.walk(path):
        for file in files:
            filePaths.append(os.path.join(root, file))
    return filePaths


def saveTupledListToFile(listToSave: list[()], toBeNamedFilePath: str,
                         counterVariable: int) -> None:
    """
    Saves a list of tuples to a file.
    :param listToSave:
    :param toBeNamedFile:
    :param directoryPath:
    :return:
    """

    outputDirectory = os.path.dirname(toBeNamedFilePath + "-" + str(counterVariable) + ".txt")

    if not os.path.exists(outputDirectory):
        os.makedirs(outputDirectory)

    with open(toBeNamedFilePath + "-" + str(counterVariable) + ".txt", "w", encoding="utf-8") as file:
        for tuple in listToSave:
            builtString = ""
            for element in tuple:
                if type(element) is str:
                    element = f"\"{element}\""
                builtString += str(element) + " "
            file.write(builtString + "\n")


def spellingBee(transcriber: AbstractSpeechTranscriber.AbstractSpeechTranscriber, specialCharacter: str, publisher,
                requiredErrorCount: int, logger: LogHandler, pathToAudioFileDirectory: str = None,
                saveResultsAsFilename: str = None, selectedRainbowword: int = None) -> bool:
    """
    Executes the spelling bee test of the CAM ICU. Returns true if more than 2 errors were made, false otherwise.
    :param pathToAudioFileDirectory:
    :param logger:
    :param transcriber:
    :param specialCharacter:
    :param publisher:
    :param requiredErrorCount:
    :return:
    """
    global globalSpellingBeeFileCounter

    # load content from json file -- TODO: Loading content from the file doesn't work in ROS, so here it is a workaround
    # with open("rainbowwords.json") as json_file:
    # rainbowwords = list(json.load(json_file)["words"])
    # json_file.close()

    rainbowWordJson = '["Casablanca","Ananasbaum"]'

    rainbowWords = list(json.loads(rainbowWordJson))

    number: int = -1
    if selectedRainbowword is not None:
        if selectedRainbowword in range(len(rainbowWords)):
            number = selectedRainbowword

    if number == -1:
        logger.log(f"Es stehen folgende Wörter für den Buchstabiertest zur Auswahl:")
        for i in range(len(rainbowWords)):
            logger.log(f"{str(i)}: {rainbowWords[i]}")
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

    # get list of error letters in rainbow word that correspond to the error answers
    errorLetters = []
    for i in range(len(errorAnswers)):
        errorLetters += [rainbowWords[number][errorAnswers[i]]]

    logger.log(f"Es werden folgende Buchstaben manipuliert: {errorLetters}")

    pathsToAudioFiles: list[str] = []
    if pathToAudioFileDirectory is not None:
        pathsToAudioFiles: list[str] = getFilePathsOfFilesInSubdirectories(pathToAudioFileDirectory)
        if len(pathsToAudioFiles) != len(rainbowWords[number]):
            logger.log(
                f"Die Anzahl der Audio Dateien ({len(pathsToAudioFiles)}) stimmt nicht mit der Anzahl der Buchstaben ({len(rainbowWords[number])}) überein!",
                LOGLEVEL.ERROR)
            return False

    listOfCorrectlyIdentifiedCharacters: list[(str, bool)] = []
    for i in range(len(rainbowWords[number])):
        if len(pathsToAudioFiles) > 0:
            transcriber.setAudioFile(pathsToAudioFiles[i])
        recognizedWord = transcriber.transcribePartially().upper()
        isActionManipulated = True if i in errorAnswers else False
        isCharacterRecognized = True if recognizedWord.upper()[0] == specialCharacter.upper() else False
        shouldCharacterBeRecognized = True if rainbowWords[number][i].upper() == specialCharacter.upper() else False

        listOfCorrectlyIdentifiedCharacters += [
            (rainbowWords[number][i].upper(), isCharacterRecognized == shouldCharacterBeRecognized)]

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

    if not isRosUsed:
        if saveResultsAsFilename is None:
            saveResultsAsFilename = "../automatedResults/spellingBeeResults/tmp/tmpfile"
        logger.log("Ergebnisse werden abgespeichert ...")
        with threadingLock:
            saveTupledListToFile(listToSave=listOfCorrectlyIdentifiedCharacters,
                                 toBeNamedFilePath=saveResultsAsFilename,
                                 counterVariable=globalSpellingBeeFileCounter)
            globalSpellingBeeFileCounter += 1

    if requiredErrorCount > 2:
        logger.log(f"{requiredErrorCount} Fehler aufgetreten. Delir möglich!")
        return True
    logger.log(f"{requiredErrorCount} Fehler aufgetreten. Kein Delir!")
    return False


def logicQuestions(transcriber: AbstractSpeechTranscriber.AbstractSpeechTranscriber, publisher,
                   requiredErrorCount: int, numberOfQuestions: int, logger: LogHandler,
                   pathToAudioFileDirectory: str = None, saveResultsAsFilename: str = None) -> bool:
    """
    Executes the logic questions test of the CAM ICU. Returns true if more than 1 error was made, false otherwise.
    :param saveResultsAsFilename:
    :param logger:
    :param transcriber:
    :param publisher:
    :param requiredErrorCount:
    :param numberOfQuestions:
    :return:
    """
    global globalLogicQuestionsFileCounter

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

    pathsToAudioFiles: list[str] = []
    if pathToAudioFileDirectory is not None:
        pathsToAudioFiles: list[str] = getFilePathsOfFilesInSubdirectories(pathToAudioFileDirectory)
        if len(pathsToAudioFiles) != numberOfQuestions:
            logger.log(
                f"Die Anzahl der Audio Dateien ({len(pathsToAudioFiles)}) stimmt nicht mit der Anzahl der gewünschten Fragen ({numberOfQuestions}) überein!",
                LOGLEVEL.ERROR)
            return False

    listOfCorrectlyIdentifiedQuestions: list[(str, str, float, str)] = []
    for i in range(numberOfQuestions):
        logger.log(f"Logikfrage {i + 1} von {numberOfQuestions}")
        isActionManipulated = True if i in errorAnswers else False

        if len(pathToAudioFileDirectory) > 0:
            transcriber.setAudioFile(pathsToAudioFiles[i])

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
        roundedConfidence = round(questionMiddledConfidences[matchingQuestion], 2)
        logger.log(
            f"Antwort auf Frage \"{initialText}\" ist mit einer Konfidenz von {roundedConfidence}:\n\"{questions[matchingQuestion]['question']}\" -> {questions[matchingQuestion]['answer']}")

        listOfCorrectlyIdentifiedQuestions += [(initialText, questions[matchingQuestion]['question'], roundedConfidence,
                                                questions[matchingQuestion]['answer'])]

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

    if not isRosUsed:
        if saveResultsAsFilename is None:
            saveResultsAsFilename = "../automatedResults/logicQuestionResults/logicQuestionResults/tmp/tmpfile"
        logger.log("Ergebnisse der Logikfragen werden abgespeichert ...")
        with threadingLock:
            saveTupledListToFile(listToSave=listOfCorrectlyIdentifiedQuestions,
                                 toBeNamedFilePath=saveResultsAsFilename,
                                 counterVariable=globalLogicQuestionsFileCounter)
            globalLogicQuestionsFileCounter += 1

    if requiredErrorCount > 1:
        logger.log(f"CAM ICU Test mit {requiredErrorCount} Fehlern durchgefallen. Delir ist vorliegend!")
        return True
    logger.log(f"CAM ICU Test mit {requiredErrorCount} Fehlern bestanden. Kein Delir vorliegend!")
    return False


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
    while transcriberWantedNumber != 1 and transcriberWantedNumber != 2 and transcriberWantedNumber != 3:
        transcriberWantedNumber = int(input("Bitte eine Zahl eingeben: "))

    if transcriberWantedNumber == 1:
        transcriber = pst.PocketSphinxTranscriber(logger=logger, isMicrophoneUsed=False)
    elif transcriberWantedNumber == 2:
        transcriber = vsk.VoskTranscriber(logger=logger)
    else:
        transcriber = nst.NetworkTranscriber(address="10.0.0.1", port=4444, logger=logger)

    # Setup CAM ICU Test
    requiredErrorQuoteForSpellingBee: int = -1
    while requiredErrorQuoteForSpellingBee < 0:
        requiredErrorQuoteForSpellingBee = int(input(
            "Bitte eine gewünschte Anzahl an Fehlern für den Buchstabierteil eingeben: "))

    numberOfLogicQuestions: int = -1
    while numberOfLogicQuestions < 0:
        numberOfLogicQuestions = int(input("Bitte eine gewünschte Anzahl an Logikfragen eingeben: "))
    requiredErrorQuoteForLogicQuestions: int = -1
    while requiredErrorQuoteForLogicQuestions < 0 or requiredErrorQuoteForLogicQuestions > numberOfLogicQuestions:
        requiredErrorQuoteForLogicQuestions = int(input(
            "Bitte eine gewünschte Anzahl an Fehlern für den Logikfragenteil eingeben: "))

    # get file paths for spellingbee and logic questions
    spellingBeeFolderNames = os.listdir("../audio/CutAndPrepared/Spelling")
    spellingBeeFolderPaths = ["../audio/CutAndPrepared/Spelling/" + x for x in spellingBeeFolderNames]
    logicQuestionFolderNames = os.listdir("../audio/CutAndPrepared/Logic")
    logicQuestionFolderPaths = ["../audio/CutAndPrepared/Logic/" + x for x in logicQuestionFolderNames]

    # setup executor thread pool

    executorPool = []
    for _ in range(0, 10):
        for i in range(len(spellingBeeFolderPaths)):
            executorPool += [threading.Thread(target=spellingBee, args=(
            transcriber, "a", publisher, requiredErrorQuoteForSpellingBee, logger, spellingBeeFolderPaths[i],
            f"../automatedResults/spellingBeeResults/{spellingBeeFolderNames[i]}", 1))]
            # break

        for i in range(len(logicQuestionFolderPaths)):
            executorPool += [threading.Thread(target=logicQuestions, args=(
            transcriber, publisher, requiredErrorQuoteForLogicQuestions, numberOfLogicQuestions, logger,
            logicQuestionFolderPaths[i], f"../automatedResults/logicQuestionResults/{logicQuestionFolderNames[i]}"))]
            # break

    for thread in executorPool:
        thread.start()

    for thread in executorPool:
        thread.join()

    logger.log("Alle Threads abgeschlossen")

    # Run CAM ICU Test
    # spellingBee(transcriber=transcriber, specialCharacter="a", publisher=publisher,
    #            requiredErrorCount=requiredErrorQuoteForSpellingBee, logger=logger,
    #            pathToAudioFileDirectory="audio/CutAndPrepared/Spelling", saveResultsAsFilename="../automatedResults/spellingBeeResults/spellingBeeResults/tmp/tmpfile", selectedRainbowword=1)
    # logicQuestions(transcriber=transcriber, publisher=publisher, requiredErrorCount=requiredErrorQuoteForLogicQuestions,
    #               numberOfQuestions=numberOfLogicQuestions, logger=logger,
    #               pathToAudioFileDirectory="audio/CutAndPrepared/Logic", saveResultsAsFilename="../automatedResults/logicQuestionResults/logicQuestionResults/")


if __name__ == '__main__':
    main()
