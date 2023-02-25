#!/usr/bin/env zsh

CATKIN_WORKSPACE="$HOME/catkin3_ws"
PROJECT_DIR="$CATKIN_WORKSPACE/src/delir"

SPRACHERKENNUNGS_PC_IP_ADDRESSE="10.3.14.243"

cd "$CATKIN_WORKSPACE"

source "$CATKIN_WORKSPACE/install/setup.zsh"

export LD_PRELOAD="/usr/lib/arm-linux-gnueabihf/libatomic.so.1.2.0"

cd "$PROJECT_DIR"

"$PROJECT_DIR/rust_audio_sender" &

AUDIO_SENDER_PID=$!

function cleanup_audio_sender {
  kill $AUDIO_SENDER_PID
}
trap cleanup_audio_sender EXIT INT

echo "Starte jetzt dis Spracherkennung auf dem anderen Rechner."
echo "Drücke danach ENTER:"

read

export PEER_IP="$SPRACHERKENNUNGS_PC_IP_ADDRESSE"

cd "$CATKIN_WORKSPACE"

roslaunch delir delir.launch


