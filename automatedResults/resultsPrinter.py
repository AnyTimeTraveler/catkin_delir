import os
import re

from matplotlib import pyplot as plt
import numpy as np

def printPlotResults() -> None:
    # Load files from logicQuestionResults/ and read data
    # sort data by file name (langsam/mittel/schnell)

    # load files from spellingBeeResults/ and read data
    # sort data by file name (langsam/mittel/schnell)

    # make diagram of logicQuestionResults

    # make diagram of spellingBeeResults

    voskOrPocketSphinx = -1
    while voskOrPocketSphinx != 0 and voskOrPocketSphinx != 1:
        voskOrPocketSphinx = int(input("Vosk-Daten (0) oder PocketSphinx-Daten (1) verwenden? : "))

    if voskOrPocketSphinx == 0:
        pathAddage = "voskResults/"
    else:
        pathAddage = "pocketSphinxResults/"



    identifierKeywords: list = ["langsam", "mittel", "schnell"]

    logicQuestionFileNamesComplete: list = os.listdir(pathAddage+"logicQuestionResults/")
    logicQuestionFileNamesSlow: list = [pathAddage+"logicQuestionResults/" + fileName for fileName in logicQuestionFileNamesComplete if identifierKeywords[0] in fileName]
    logicQuestionFileNamesMedium: list = [pathAddage+"logicQuestionResults/" + fileName for fileName in logicQuestionFileNamesComplete if identifierKeywords[1] in fileName]
    logicQuestionFileNamesFast: list = [pathAddage+"logicQuestionResults/" + fileName for fileName in logicQuestionFileNamesComplete if identifierKeywords[2] in fileName]

    combinedLogicQuestionFileNames: list[list] = [logicQuestionFileNamesSlow, logicQuestionFileNamesMedium, logicQuestionFileNamesFast]

    spellingBeeFileNamesComplete: list = os.listdir(pathAddage+"spellingBeeResults/")
    spellingBeeFileNamesSlow: list = [pathAddage+"spellingBeeResults/" + fileName for fileName in spellingBeeFileNamesComplete if identifierKeywords[0] in fileName]
    spellingBeeFileNamesMedium: list = [pathAddage+"spellingBeeResults/" + fileName for fileName in spellingBeeFileNamesComplete if identifierKeywords[1] in fileName]
    spellingBeeFileNamesFast: list = [pathAddage+"spellingBeeResults/" + fileName for fileName in spellingBeeFileNamesComplete if identifierKeywords[2] in fileName]

    combinedSpellingBeeFileNames: list[list] = [spellingBeeFileNamesSlow, spellingBeeFileNamesMedium, spellingBeeFileNamesFast]

    logicQuestionResultsSlow: dict[bool:int] = {True: 0 , False: 0}
    logicQuestionResultsMedium: dict[bool:int] = {True: 0 , False: 0}
    logicQuestionResultsFast: dict[bool:int] = {True: 0 , False: 0}

    combinedLogicQuestionResults: list[dict] = [logicQuestionResultsSlow, logicQuestionResultsMedium, logicQuestionResultsFast]

    logicQuestionDetectionSpeedSlow: list[float] = []
    logicQuestionDetectionSpeedMedium: list[float] = []
    logicQuestionDetectionSpeedFast: list[float] = []

    combinedLogicQuestionDetectionSpeed: list[list[float]] = [logicQuestionDetectionSpeedSlow, logicQuestionDetectionSpeedMedium, logicQuestionDetectionSpeedFast]


    spellingBeeResultsSlow: dict[bool:int] = {True: 0 , False: 0}
    spellingBeeResultsMedium: dict[bool:int] = {True: 0 , False: 0}
    spellingBeeResultsFast: dict[bool:int] = {True: 0 , False: 0}

    combinedSpellingBeeResults: list[dict] = [spellingBeeResultsSlow, spellingBeeResultsMedium, spellingBeeResultsFast]

    spellingBeeDetectionSpeedSlow: list[float] = []
    spellingBeeDetectionSpeedMedium: list[float] = []
    spellingBeeDetectionSpeedFast: list[float] = []

    combinedSpellingBeeDetectionSpeed: list[list[float]] = [spellingBeeDetectionSpeedSlow, spellingBeeDetectionSpeedMedium, spellingBeeDetectionSpeedFast]


    for i in range(len(combinedLogicQuestionFileNames)):
        for fileName in combinedLogicQuestionFileNames[i]:
            with open(fileName, "r") as f:
                lines = f.readlines()
                for line in lines:
                    foundStrings = re.findall(r'"([^"]*)"', line)
                    initialQuestion = foundStrings[0]
                    detectedQuestion = foundStrings[1]
                    answer = True if foundStrings[2] == "y" else False

                    foundNumber = re.findall(r'\'([^\']*)\'', line)
                    confidenceFloat = float(foundNumber[0])
                    latencyFloat = float(foundNumber[1])

                    if confidenceFloat > 0.75:
                        combinedLogicQuestionResults[i][True] += 1
                    else:
                        combinedLogicQuestionResults[i][False] += 1

                    combinedLogicQuestionDetectionSpeed[i].append(latencyFloat)



    for i in range(len(combinedSpellingBeeFileNames)):
        for fileName in combinedSpellingBeeFileNames[i]:
            with open(fileName, "r") as f:
                lines = f.readlines()
                for line in lines:
                    if "True" in line:
                        combinedSpellingBeeResults[i][True] += 1
                    else:
                        combinedSpellingBeeResults[i][False] += 1
                    foundNumber = re.findall(r'\'([^\']*)\'', line)
                    foundNumber = float(foundNumber[0])
                    combinedSpellingBeeDetectionSpeed[i].append(foundNumber)



    # Logic Question Results
    # generate diagram for logicQuestionResults

    width = 24
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(width, width))



    spellingBeeTrueValues = [combinedSpellingBeeResults[0][True], combinedSpellingBeeResults[1][True], combinedSpellingBeeResults[2][True]]
    spellingBeeFalseValues = [combinedSpellingBeeResults[0][False], combinedSpellingBeeResults[1][False], combinedSpellingBeeResults[2][False]]

    totalSpellingValues = [i + j for i, j in zip(spellingBeeTrueValues, spellingBeeFalseValues)]

    spellingBeeTrueRelativeValues = [i / j * 100 for i, j in zip(spellingBeeTrueValues, totalSpellingValues)]
    spellingBeeFalseRelativeValues = [i / j * 100 for i, j in zip(spellingBeeFalseValues, totalSpellingValues)]
    xArrangement: list = [0,1.2,2.4]


    green = "mediumseagreen"
    red = "indianred"

    ax1.set_title("Buchstabiertest")
    ax1.set_xlabel("Geschwindigkeit")
    ax1.set_ylabel("Prozent korrekt identifizierter Buchstaben")
    ax1.bar(xArrangement, spellingBeeTrueRelativeValues, alpha=0.5, tick_label=identifierKeywords, color=green)
    ax1.bar(xArrangement, spellingBeeFalseRelativeValues, alpha=0.5, tick_label=identifierKeywords, color=red, bottom=spellingBeeTrueRelativeValues)
    ax1.grid(axis="y", linestyle="--", alpha=0.5)


    logicQuestionTrueValues = [combinedLogicQuestionResults[0][True], combinedLogicQuestionResults[1][True], combinedLogicQuestionResults[2][True]]
    logicQuestionFalseValues = [combinedLogicQuestionResults[0][False], combinedLogicQuestionResults[1][False], combinedLogicQuestionResults[2][False]]

    totalLogicValues = [i + j for i, j in zip(logicQuestionTrueValues, logicQuestionFalseValues)]

    logicQuestionTrueRelativeValues = [i / j * 100 for i, j in zip(logicQuestionTrueValues, totalLogicValues)]
    logicQuestionFalseRelativeValues = [i / j * 100 for i, j in zip(logicQuestionFalseValues, totalLogicValues)]

    ax2.set_title("Logikfragen")
    ax2.set_xlabel("Geschwindigkeit")
    ax2.set_ylabel("Prozent korrekt identifizierter Fragen")
    ax2.bar(xArrangement, logicQuestionTrueRelativeValues, alpha=0.5, tick_label=identifierKeywords, color=green)
    ax2.bar(xArrangement, logicQuestionFalseRelativeValues, alpha=0.5, tick_label=identifierKeywords, color=red, bottom=logicQuestionTrueRelativeValues)
    ax2.grid(axis="y", linestyle="--", alpha=0.5)

    ax3.set_title("Buchstabiertest")
    ax3.set_xlabel("Geschwindigkeit")
    ax3.set_ylabel("Durchschnittliche Latenzzeit in s")
    ax3.boxplot(combinedSpellingBeeDetectionSpeed, labels=identifierKeywords)
    ax3.grid(axis="y", linestyle="--", alpha=0.5)

    ax4.set_title("Logikfragen")
    ax4.set_xlabel("Geschwindigkeit")
    ax4.set_ylabel("Durchschnittliche Latenzzeit in s")
    ax4.boxplot(combinedLogicQuestionDetectionSpeed, labels=identifierKeywords)
    ax4.grid(axis="y", linestyle="--", alpha=0.5)

    fig.show()


if __name__ == "__main__":
    printPlotResults()