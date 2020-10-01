#!/usr/bin/sh 

cd ../client && yarn start &
roscd && cd ../ && roslaunch backend final.launch &
