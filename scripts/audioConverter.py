#get all files in subdirectories of ../audio and replace all .mp3 with .wav
import os
import subprocess
import sys
import glob
import shutil

def convertToWav():
    #get all files in subdirectories of ../audio and replace all .mp3 with .wav
    for file in glob.glob("../audio/CutAndPrepared/**/*.mp3", recursive=True):
        print(file)
        subprocess.call(["ffmpeg", "-i", file, file.replace(".mp3", ".wav")])
        os.remove(file)

if __name__ == "__main__":
    convertToWav()