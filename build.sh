#!/usr/bin/env zsh

CATKIN_WORKSPACE="$HOME/catkin3_ws"

source "$CATKIN_WORKSPACE/install"

cd "$CATKIN_WORKSPACE"

catkin build delir

