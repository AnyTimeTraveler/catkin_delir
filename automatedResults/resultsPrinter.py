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

    identifierKeywords: list = ["langsam", "mittel", "schnell"]

    logicQuestionFileNamesComplete: list = os.listdir("logicQuestionResults/")
    logicQuestionFileNamesSlow: list = ["logicQuestionResults/" + fileName for fileName in logicQuestionFileNamesComplete if identifierKeywords[0] in fileName]
    logicQuestionFileNamesMedium: list = ["logicQuestionResults/" + fileName for fileName in logicQuestionFileNamesComplete if identifierKeywords[1] in fileName]
    logicQuestionFileNamesFast: list = ["logicQuestionResults/" + fileName for fileName in logicQuestionFileNamesComplete if identifierKeywords[2] in fileName]

    combinedLogicQuestionFileNames: list[list] = [logicQuestionFileNamesSlow, logicQuestionFileNamesMedium, logicQuestionFileNamesFast]

    spellingBeeFileNamesComplete: list = os.listdir("spellingBeeResults/")
    spellingBeeFileNamesSlow: list = ["spellingBeeResults/" + fileName for fileName in spellingBeeFileNamesComplete if identifierKeywords[0] in fileName]
    spellingBeeFileNamesMedium: list = ["spellingBeeResults/" + fileName for fileName in spellingBeeFileNamesComplete if identifierKeywords[1] in fileName]
    spellingBeeFileNamesFast: list = ["spellingBeeResults/" + fileName for fileName in spellingBeeFileNamesComplete if identifierKeywords[2] in fileName]

    combinedSpellingBeeFileNames: list[list] = [spellingBeeFileNamesSlow, spellingBeeFileNamesMedium, spellingBeeFileNamesFast]

    logicQuestionResultsSlow: dict[bool:int] = {True: 0 , False: 0}
    logicQuestionResultsMedium: dict[bool:int] = {True: 0 , False: 0}
    logicQuestionResultsFast: dict[bool:int] = {True: 0 , False: 0}

    combinedLogicQuestionResults: list[dict] = [logicQuestionResultsSlow, logicQuestionResultsMedium, logicQuestionResultsFast]

    spellingBeeResultsSlow: dict[bool:int] = {True: 0 , False: 0}
    spellingBeeResultsMedium: dict[bool:int] = {True: 0 , False: 0}
    spellingBeeResultsFast: dict[bool:int] = {True: 0 , False: 0}

    combinedSpellingBeeResults: list[dict] = [spellingBeeResultsSlow, spellingBeeResultsMedium, spellingBeeResultsFast]

    for i in range(len(combinedLogicQuestionFileNames)):
        for fileName in combinedLogicQuestionFileNames[i]:
            with open(fileName, "r") as f:
                lines = f.readlines()
                for line in lines:
                    foundStrings = re.findall(r'"([^"]*)"', line)
                    initialQuestion = foundStrings[0]
                    detectedQuestion = foundStrings[1]
                    answer = True if foundStrings[2] == "y" else False

                    foundNumber = re.findall(r'\b(\d+(\.\d{1,2})?|\.\d{1,2})\b', line)
                    foundNumber.sort(key=len, reverse=True)
                    foundNumber = float(foundNumber[0][0])

                    if foundNumber > 0.75:
                        combinedLogicQuestionResults[i][True] += 1
                    else:
                        combinedLogicQuestionResults[i][False] += 1



    for i in range(len(combinedSpellingBeeFileNames)):
        for fileName in combinedSpellingBeeFileNames[i]:
            with open(fileName, "r") as f:
                lines = f.readlines()
                for line in lines:
                    if "True" in line:
                        combinedSpellingBeeResults[i][True] += 1
                    else:
                        combinedSpellingBeeResults[i][False] += 1



    # Logic Question Results
    # generate diagram for logicQuestionResults

    width = 11
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(width, width / 2))



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

    fig.show()


if __name__ == "__main__":
    printPlotResults()