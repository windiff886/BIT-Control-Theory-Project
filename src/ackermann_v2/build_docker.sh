#!/bin/bash
IMAGE_NAME=gazebo_ackermann_steering_vehicle
IMAGE_TAG=harmonic

docker build -t $IMAGE_NAME:$IMAGE_TAG . 