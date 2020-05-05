#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
echo "Let's do it"

roslaunch beh_benchmark beh_benchmark.launch veh:=$VEHICLE_NAME
