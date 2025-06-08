#!/bin/bash

# Creates required temp directory for GUI applications - needed by Linux desktop standards
# Set up XDG runtime directory
export XDG_RUNTIME_DIR=/tmp/runtime-$USER
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# Any command you run after this script runs in the environment set up by the script.
exec "$@"