Build the modified post-processor by running:
* `docker build -t duckietown/post-processing:v1 .`
Then run it with:
* `docker run --name post_processor -it --rm -e INPUT_BAG_PATH=/data/BAGNAME_localization -e OUTPUT_BAG_PATH=/data/processed_BAGNAME_localization.bag -e ROS_MASTER_URI=http://192.168.1.97:11311 -v path_to_bag_folder:/data duckietown/post-processing:v1`
