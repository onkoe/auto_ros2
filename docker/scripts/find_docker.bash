#!/usr/bin/bash

# stop script on error
set -e

if command -v podman &> /dev/null; then
    export DOCKER="podman"
    echo "Found Podman as an alternative to Docker. We'll use that instead!"
else
    export DOCKER="docker"
    echo "Attempting to use raw Docker..."
fi
