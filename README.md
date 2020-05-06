# behaviour_benchmarking

## Requiries

* Requires: [Duckiebot (correctly assembled, initialized and calibrated)](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/building_duckiebot_c0.html)
* Requires: [Localization standoff](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_autobot_specs.html)
* Requires: [Autobot](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_autobot_specs.html)
* Requires: [2 straight and 4 curved tiles](https://docs.duckietown.org/daffy/opmanual_duckietown/out/dt_ops_appearance_specifications.html)
* Requires: [At least 4 Watchtowers in WT19B configuration](https://docs.duckietown.org/daffy/opmanual_autolab/out/watchtower_hardware.html)
* Requires: [X Ground April tags](https://github.com/duckietown/docs-resources_autolab/blob/daffy/AprilTags/AprilTags_localization_ID300-399.pdf)
* Results: Lane Following Benchmarking

## HW set up
Needed?
## Loop Assembly and map making
First assemble the tiles to a 2x3 closed loop as seen below.

![linus_loop](/media/linus_loop.png)

Further place the Watchtowers around the loop susch that each part of it is seen by at least one Watchtower. To get really accurate results it is recommended to place them as seen below

![WT position](/media/Linus_Loop_WT_correct.png).

Fork and clone the [Duckietown-world repository](https://github.com/duckietown/duckietown-world) and follow the instructions found [here](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_map_making.html) to create your own map. Or simply add the [linus_loop folder](/linus_loop_data/linus_loop) into the _visualization/maps_ folder of your duckietown-world repository and the [linus_loop yaml file](/linus_loop_data/linus_loop_no_at.yaml) into the _src/duckietown_world/data/gd1/maps_ folder of your duckietown-world repository.

Then place at least 8 Ground April Tags roughly as seen in the images such that each watchtower sees at least 2 Apriltags.
After follow the instructions found [here](https://docs.duckietown.org/daffy/opmanual_autolab/out/localization_apriltags_specs.html) to add their exact position to the map.

## Localization Set up

Set up the offline localization following the instructions found [here](https://docs.duckietown.org/daffy/opmanual_autolab/out/localization_demo.html)  (Steps 3.1-3.5).

## Software preparation
* TODO: Instructions how to set Gain=1
* Fork and clone the [behaviour_benchmarking](https://gitlab.com/llingg/behaviour_benchmarking/-/tree/master) repository
* Fork and clone the [behaviour-benchmarking](https://github.com/llingg/behaviour-benchmarking) repository
* Be sure that `dt-core`, `dt-car-interface`, `dt-duckiebot-interface`, `dt-ros-commons` images are updated. If not, pull them:
    * `docker -H BOTNAME.local pull duckietown/dt-core:daffy-arm32v7@sha256:4c7633c2041f5b7846be2346e0892c9f50987d2fd98d3479ec1a4cf378f52ee6`
    * `docker -H BOTNAME .local pull duckietown/dt-car-interface:daffy-arm32v7@sha256:e3db984157bf3a2b2d4ab7237536c17b37333711244a3206517daa187c143016`
    * `docker -H BOTNAME.local pull duckietown/dt-duckiebot-interface:daffy-arm32v7@sha256:94a9defa553d1e238566a621e084c4b368e6a9b62053b02f0eef1d5685f9ea73`
    * `docker -H BOTNAME.local pull duckietown/dt-ros-commons:daffy-arm32v7@sha256:20840df4cd5a8ade5949e5cfae2eb9b5cf9ee7c0`
* If all the images are updated you can start the following steps:

1. Make sure all old containers from the images `dt-duckiebot-interface`, `dt-car-interface`, and `dt-core` are stopped. These containers can have different names, instead look at the image name from which they are run.    

2. Start all the drivers in `dt-duckiebot-interface`:
    * `dts duckiebot demo --demo_name all_drivers --duckiebot_name BOTNAME --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy`

    and the glue nodes that handle the joystick mapping and the kinematics:
    * `dts duckiebot demo --demo_name all --duckiebot_name BOTNAME --package_name car_interface --image duckietown/dt-car-interface:daffy`

    Make sure that this worked properly.

* Within the folder _packages/light_lf_ of your behaviour-benchmarking repository:
1. You can **build** the docker container as follows:
    * `docker -H BOTNAME.local build --no-cache -t light_lf:BRANCH_NAME .`

4. After that, if there were no errors, you can **run** the light_lf:
    * `docker -H BOTNAME.local run -it --rm -v /data:/data --privileged --network=host light_lf:BRANCH_NAME`

## Add your contribution
To see if you contribution has imporved the Lanefollowing just add your contribution into the _packages/light_lf/packages_ folder and build the container again:
  * `docker -H BOTNAME.local build --no-cache -t light_lf:BRANCH_NAME .`
Then run your version of dt-core:
  * `docker -H BOTNAME.local run -it --rm -v /data:/data --privileged --network=host light_lf:BRANCH_NAME`

For example, when you have worked one the lane_filter, then simply add your entire lane_filter folder into the folder _packages/light_lf/packages_. Please make sure that you use the precise name, as then the default version of whatever package is automatically replaced by yours.
To get all the different packages in which you can make changes or work in please check [here](https://github.com/duckietown/dt-core/tree/daffy/packages).

If you would like to run indefinite_navigation instead of just lane_following, just uncomment line 3 in the light_lf.launch file found in light_lf/packages/light_lf/launch folder and comment out line 2.

In the end it is tha same as if you would simply clone the [dt-core](https://github.com/duckietown/dt-core) repository and building and running this on your Duckiebot.
However, it is suggested to develop as you wish but then for the actual Benchmarking to use the way explained above which uses a lighter version of the launch files.

## Benchmarking
First of all, for each Duckiebot involved,  run the hw_check you can find within the cloned behaviour-benchmarking repository. Therefor follow the following steps:
* Go into the folder hw_check by running:
  * `cd ~/behaviour-benchmarking/packages/hw_check`
* Build the container by running:
  * `docker -H BOTNAME.local build --no-cache -t hw_check:v1 .`
* Then run it by running:
  * `docker -H BOTNAME.local run -it --network host -v /data:/data -v /sys/firmware/devicetree:/devfs:ro hw_check:v1`


  Then follow the instructions within the terminal.

  TODO: at the moment Hat and Pi not detected! Figure out why.

* When the Docker Container has finished, visit: `http://BOTNAME.local:8082/config` and download the .yaml file with your information in the name.
* Place the .yaml file within the _data/BenchmarkXY_ folder of your behaviour_benchmarking repository.
* Furthermore, set up your own [autolab fleet rooster](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_fleet_roster.html)

Place your Duckiebot within the map.

Prepare 3 terminals:
* Terminal 1: Run the diagnostic toolbox on your Duckiebot:
    * `dts diagnostics run -G Name_BehBench_LF -d 70 --type duckiebot -H BOTNAME.local`
* Terminal 2: Start the keyboard control on your Duckiebot:
    * `dts duckiebot keyboard_control BOTNAME --base_image duckietown/dt-core:daffy-amd64`
To start lane_following press 'a' on your keyboard
* Terminal 3: Open a Docker container ros being preinstalled by running:
    * `dts cli`

    Then within this container record a rosbag that subscribes everything by running:
    * `rosbag record -a --duration=50 -O Country_University_LoopName_Date_GithubUserName_HWConfig_SWConig.bag`

After the rosbag recording as well as the Diagnostic Toolbox have finished you can stop the Duckiebot by pressing 's' on your keyboard.
Then do the follwing steps:
* Copy the recorded rosbag from the Docker container onto your local computer into the _path_to_bag_folder_ by running:
  * `sudo docker cp dts-cli-run-helper:/code/catkin_ws/src/dt-gui-tools/BAG_NAME.bag ~/path_to_bag_folder`
or generally:
  * `sudo docker cp Docker_Container_Name:/place_within_container/where_bag_was_recorded/BAG_NAME.bag ~/path_to_bag_folder`
* Make sure thet the bag is readable by running:
  * `sudo chmod 777 BAG_NAME.bag`
* Run the post_processor by running:
  * `docker run --name post_processor -dit --rm -e INPUT_BAG_PATH=/data/BAG_NAME.BAG -e OUTPUT_BAG_PATH=/data/processed_BAG_NAME.BAG -e ROS_MASTER_URI=http://YOUR_IP:11311 -v PATH_TO_BAG_FOLDER:/data duckietown/post-processor:daffy-amd64`

  You need to know where your bag is. The folder containing it is referred as PATH_TO_BAG_FOLDER in the command above. it is recommended to create new separate folders for each Benchmark (with date and/or sequence number).
When the container stops, then you should have a new bag called processed_BAG_NAME.BAG inside of your PATH_TO_BAG_FOLDER. (This can take more than a minute, please be patient)
* Remember from [Unit B-4 - Autolab map](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_map_making.html) that you created a map. Now is the time to remember on which fork you pushed it (the default is duckietown), and what name you gave to your map. The map file needs to be in the same folder as the rest of the maps. They are respectively the YOUR_FORK_NAME and YOUR_MAP_NAME arguments in the following command line:
* `docker run --rm  -e  ATMSGS_BAG=/data/processed_BAG_NAME.BAG -e OUTPUT_DIR=/data -e ROS_MASTER=YOUR_HOSTNAME -e ROS_MASTER_IP=YOUR_IP --name graph_optimizer -v PATH_TO_BAG_FOLDER:/data -e DUCKIETOWN_WORLD_FORK=YOUR_FORK_NAME -e MAP_NAME=YOUR_MAP_NAME duckietown/cslam-graphoptimizer:daffy-amd64`

  A _.yaml_ file will be stored in the folder PATH_TO_BAG_FOLDER.

* Visit [dashboard](https://dashboard.duckietown.org/) and login using your Duckietown token. Then navigate to _Diagnostics_ and in the drop down menue _Group_ select _Name_BehBench_LF_ and in the drop down menu _Time_ the corresponding time when you ran the Benchmark. After add the data by pressing onto the green plus and download the _.json_ file by pressing the Download log button.

* Place the .yaml file created by the graphoptimizer into the _data/BenchmarkXY/yaml_ folder and the .json file downloaded from the dashboard into the  _data/BenchmarkXY_ folder of your behaviour_benchmarking repository.
* Create a virtual environment as you already did for when you added the map to your duckietown-world repository or when you added the exact position of the ground april tags. However, this time, please create this virtual environment within your cloned behaviour_benchmarking repository by following the instructions below:
1. First, if not already done, install venv by running:
   * `sudo apt install -y python3-venv`
2. Then, cd into your behaviour_benchmarking repository, and create the venv:
   * `cd ~/behaviour_benchmarking`
   * `python3.7 -m venv duckietown-world-venv`
   * `source duckietown-world-venv/bin/activate`
3. Now, you can setup duckietown-world. Inside of the virtual environment (you should see “(duckietown-worl-venv)” in front of your prompt line), please run:
   * `python3 -m pip install --upgrade pip`
   * `python3 -m pip install -r requirements.txt`
   * `python3 -m pip install jupyter`
   * `python3 setup.py develop --no-deps`
4. Then start the notebook:
   * `jupyter notebook`

    If you encounter any issues with the steps above, please click [here](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_map_making.html) for more detailed instructions.

5. Navigate to the Notebook TODO

### Metrics
Arithmetic mean:
$$\bar{x}=\frac{1}{n} \sum_{i=1}^n x_i $$

Standard deviation:
$$s=\sqrt{\frac{1}{N}\sum_{i=1}^N(x_i-\bar{x})^2}$$

### Termination Criteria
