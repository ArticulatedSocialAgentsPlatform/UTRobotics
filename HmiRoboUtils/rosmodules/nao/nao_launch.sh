#!/bin/bash

LD_LIBRARY_PATH=~/naoqi/naoqi-sdk-1.12-linux32/lib/:/opt/ros/fuerte/lib:/opt/ros/fuerte/lib:/opt/ros/fuerte/lib: NAO_IP=127.0.0.1 roslaunch nao_driver nao_driver_sim.launch
