# Analyze Rosbag

This package allows you to analyse and extract data from a bag recorded directly on the Duckiebot during Benchmarking.

At the moment the script is set up for all kinds of Lane Following Benchmarks. It will read the bag and spit out json files containing information about:
- updated frequency of the nodes that are recorded
- latency up to and including the detector node
- global constants that were set
- number of segments detected at each time
- pose (offset and heading) estimation of the Duckiebot

To build this package run:
- `dts devel build -f --arch amd64`

Then run it with:
- `docker run -v path_to_bag_folder:/data -e DUCKIEBOT=AUTOBOT_NAME -e BAGNAME=BAGNAME -it --rm duckietown/behaviour-benchmarking:v1-amd64`

This script runs even if only rousout is recorded in the bag. 

The ros.py script is ready for analysing the rosbag recorded during the Lane Following Benchmark.

However, for any other benchmark simply add your contribution by adding your code in the ros.py script or make a copy of the script and extract the additional data needed for your Benchmark and then make sure to launch your version in the launch file. 

Like this it is easy to extract data from any kind of nodes you subscribed to, just follow the same principle as you find within the Lane Following version.
Make sure that you also recorded the topics you want to extract information from when you ran the experiment, otherwise it won't give you any results.



