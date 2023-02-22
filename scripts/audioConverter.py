# get all files in subdirectories of ../audio and replace all .mp3 with .wav
import os
import shutil
import subprocess
import glob
from pydub import AudioSegment


def convertToWav():
    # get all files in subdirectories of ../audio and replace all .mp3 with .wav
    for file in glob.glob("../audio/CutAndPrepared/**/*.mp3", recursive=True):
        print(file)

        subprocess.call(["ffmpeg", "-i", file, file.replace(".mp3", ".wav")])
        os.remove(file)


def convertToMono():
    for root, directories, files in os.walk("../audio/CutAndPrepared/"):
        for file in files:
            if file.endswith('.wav'):
                # Load the WAV file using pydub
                sound = AudioSegment.from_wav(os.path.join(root, file))

                # Convert the stereo audio to mono
                sound = sound.set_channels(1)

                # Export the mono audio as a temporary WAV file
                temp_file = os.path.join(root, 'temp.wav')
                sound.export(temp_file, format='wav')

                # Delete the original WAV file
                os.remove(os.path.join(root, file))

                # Move the temporary WAV file to the location of the original file
                shutil.move(temp_file, os.path.join(root, file))





if __name__ == "__main__":
    convertToMono()
    # convertToWav()
