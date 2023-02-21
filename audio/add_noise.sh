#!/usr/bin/env sh

NOISE_AMOUNT="0.002"


# Define the add_noise function
add_noise() {
  # Add your code here to add noise to the file
  echo "Adding noise to $1"

  # Extract the directory path and filename from the argument
  dir_path=$(dirname "$1")
  filename=$(basename "$1")

  # Remove the file extension from the filename
#  filename_no_ext="$(echo "$filename" | cut -d'.' -f 1)"

  newdir_path="$(echo "$dir_path" | sed 's/CutAndPrepared/WithNoise/')"

  mkdir -p "$newdir_path"
  # Print the directory path and filename without extension
#  echo "$dir_path"
#  echo "$filename_no_ext"

  # Create noise
  sox "$1" ./noise.wav synth whitenoise vol $NOISE_AMOUNT

  # Add created noise
  sox -m "$1" ./noise.wav "${newdir_path}/$filename"
#  sox -m "$1" ./noise.wav "${dir_path}/${filename_no_ext}.wav"
}

# Use find to locate all files ending in .wav recursively
find . -type f -name "*.wav" | while read file; do
  # Call the add_noise function on each file
  add_noise "$file"
done

rm ./noise.wav
