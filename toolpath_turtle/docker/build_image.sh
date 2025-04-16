#!/usr/bin/env bash

echo -e "Building ros_introduction:lastest image"

DOCKER_BUILDKIT=1 \
docker build --pull --rm -f ./.docker/Dockerfile \
--build-arg BUILDKIT_INLINE_CACHE=1 \
--tag ros_introduction:latest .