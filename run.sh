#!/bin/bash

DOCKER_IMAGE="camera_image:local"
DOCKER_NETWORK="scripts_my_bridge_network"


if ! command -v docker &> /dev/null
then
    echo "not install docker"
else    
    if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep 'camera_image:local'; then
        echo "camera_image:local not found. Building the Docker image...."
        docker build -t camera_image:local .
    else
        echo "camera_image:local already exists."
    fi
fi


docker run -it --rm \
        --network $DOCKER_NETWORK \
        -v $(pwd)/src/camera:/workspaces/src/camera \
        $DOCKER_IMAGE \
        /bin/bash