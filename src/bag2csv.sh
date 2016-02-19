#!/bin/bash

rostopic echo -b "$1" /coef -p > coef.csv
rostopic echo -b "$1" /imu -p > imu.csv