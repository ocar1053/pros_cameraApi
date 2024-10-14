#!/bin/bash

DOCKER_IMAGE="ghcr.io/screamlab/pros_cameraapi:latest"
DOCKER_NETWORK="scripts_my_bridge_network"


# Create a network if it doesn't exist
if [ -z "$(docker network ls --filter name=$DOCKER_NETWORK --quiet)" ]; then
    docker network create $DOCKER_NETWORK
fi


docker run -it --rm \
        --network $DOCKER_NETWORK \
        -v $(pwd)/src/camera:/workspaces/src/camera \
        $DOCKER_IMAGE \
        /bin/bash