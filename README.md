# Benchmarking: Introduction {#sec:benchmarking_introduction level=sec status=ready}

## Requiries

* Requires: [Duckiebot (correctly assembled, initialized and calibrated)](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/building_duckiebot_c0.html)
* Requires: [Localization standoff](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_autobot_specs.html)
* Requires: [Autobot](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_autobot_specs.html)
* Requires: [2 straight and 4 curved tiles](https://docs.duckietown.org/daffy/opmanual_duckietown/out/dt_ops_appearance_specifications.html)
* Requires: [At least 4 Watchtowers in WT19B configuration](https://docs.duckietown.org/daffy/opmanual_autolab/out/watchtower_hardware.html)
* Requires: [X Ground April tags](https://github.com/duckietown/docs-resources_autolab/blob/daffy/AprilTags/AprilTags_localization_ID300-399.pdf)
* Results: Lane Following Benchmarking

## Duckiebot Hardware set up
Below a few things you have to be careful with:

* Be careful with placing the April Tag in the very center of the DB as the localization system that measures the Ground Truth expects the April Tag to be exactly in the center! Future work potential!
* Make sure that standoff has an April Tag with a different name as your Duckiebot! So if you use the Autobot18 April Tag for example, make sure your duckiebot is named differently!
* Set the Gain of your Duckiebot to 1.0 by following the instructions found [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/wheel_calibration.html)


## Loop Assembly and map making
Pleas note that the loop used for these benchmarks does NOT respect the Duckietown specifications and rules as it is not allowed to have two lanes next to each other without any visual barrier.
However as all the experiments worked out fine and as the space was limited, the Benchmarks were done on this loop anyways.
In the future it is strongly recommended to change the settings to a 3x3 loop that does respect the [Duckietown specifications](https://docs.duckietown.org/daffy/opmanual_duckietown/out/dt_ops_appearance_specifications.html)

First assemble the tiles to a 2x3 closed loop as seen below.

![linus_loop](/media/linus_loop.png){ width=50% }

Further place the Watchtowers around the loop such that each part of it is seen by at least one Watchtower. To get really accurate results it is recommended to place them as seen below.

![WT position](/media/Linus_Loop_WT_correct.png){ width=50% }

Please note that it is recommended to check what the watchtowers actually just see the loop. Therefor place them in a way such that they see as little from the outside of the loop as possible.

Fork and clone the [Duckietown-world repository](https://github.com/duckietown/duckietown-world) and follow the instructions found [here](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_map_making.html) to create your own map. Or simply add the [linus_loop folder](/linus_loop_data/linus_loop) into the _visualization/maps_ folder of your duckietown-world repository and the [linus_loop yaml file](/linus_loop_data/linus_loop_no_at.yaml) into the _src/duckietown_world/data/gd1/maps_ folder of your duckietown-world repository.

Then place at least 8 Ground April Tags roughly as seen in the images such that each watchtower sees at least 2 Apriltags.
After follow the instructions found [here](https://docs.duckietown.org/daffy/opmanual_autolab/out/localization_apriltags_specs.html) to add their exact position to the map.

## Localization Set up

Set up the offline localization following the instructions found [here](https://docs.duckietown.org/daffy/opmanual_autolab/out/localization_demo.html)  (Steps 3.1-3.5).

## Software preparation
* On your local computer create a folder called `bag`
* (Fork and) clone the [behaviour_benchmarking](https://gitlab.com/llingg/behaviour_benchmarking/-/tree/master) repository
* (Fork and) clone the [behaviour-benchmarking](https://github.com/llingg/behaviour-benchmarking) repository
* Make sure that your dts command version is daffy (ToDo: how do I guarantee that diagnostics did not change?? duckietown shell commands commit 'commit 62809665b108832cdf58544ebb1d7a1d5ed997fc')
* Be sure that `dt-core`, `dt-car-interface`, `dt-duckiebot-interface`, `dt-ros-commons` images are updated. If not, pull them:
    * `docker -H BOTNAME.local pull duckietown/dt-core:daffy-arm32v7@sha256:4c7633c2041f5b7846be2346e0892c9f50987d2fd98d3479ec1a4cf378f52ee6`
    * `docker -H BOTNAME.local pull duckietown/dt-car-interface:daffy-arm32v7@sha256:e3db984157bf3a2b2d4ab7237536c17b37333711244a3206517daa187c143016`
    * `docker -H BOTNAME.local pull duckietown/dt-duckiebot-interface:daffy-arm32v7@sha256:94a9defa553d1e238566a621e084c4b368e6a9b62053b02f0eef1d5685f9ea73`
    * `docker -H BOTNAME.local pull duckietown/dt-ros-commons:daffy-arm32v7@sha256:20840df4cd5a8ade5949e5cfae2eb9b5cf9ee7c0`
* If all the images are updated you can start the following steps:

1. Make sure all old containers from the images `dt-duckiebot-interface`, `dt-car-interface`, and `dt-core` are stopped. These containers can have different names, instead look at the image name from which they are run.    

2. Start all the drivers in `dt-duckiebot-interface`:
    * `dts duckiebot demo --demo_name all_drivers --duckiebot_name BOTNAME --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy`

    and the glue nodes that handle the joystick mapping and the kinematics:
    * `dts duckiebot demo --demo_name all --duckiebot_name BOTNAME --package_name car_interface --image duckietown/dt-car-interface:daffy`

    Make sure that this worked properly.

3. Within the folder _packages/light_lf_ of the behaviour-benchmarking repository:
     1. You can **build** the docker container as follows:
    * `docker -H BOTNAME.local build --no-cache -t light_lf:v1 .`

   2. After that, if there were no errors, you can **run** the light_lf:
    * `docker -H BOTNAME.local run -it  --name behaviour_benchmarking --rm -v /data:/data --privileged --network=host light_lf:v1`

## Add your contribution
To see if you contribution has imporved the Lanefollowing just add your contribution into the _packages/light_lf/packages_ folder and build the container again:
  * `docker -H BOTNAME.local build --no-cache -t light_lf:BRANCH_NAME .`
Then run your version of dt-core:
  * `docker -H BOTNAME.local run -it --name behaviour_benchmarking --rm -v /data:/data --privileged --network=host light_lf:BRANCH_NAME`

For example, when you have worked one the lane_filter, then simply add your entire lane_filter folder into the folder _packages/light_lf/packages_. Please make sure that you use the precise name, as then the default version of whatever package is automatically replaced by yours.
To get all the different packages in which you can make changes or work in please check [here](https://github.com/duckietown/dt-core/tree/daffy/packages).

If you would like to run indefinite_navigation instead of just lane_following, just uncomment line 3 in the light_lf.launch file found in light_lf/packages/light_lf/launch folder and comment out line 2.

In the end it is the same as if you would simply clone the [dt-core](https://github.com/duckietown/dt-core) repository and building and running this on your Duckiebot.
However, it is suggested to develop as you wish and then for the actual Benchmarking to use the way explained above which uses a lighter version of the launch files. This guarantees the benchmarks to be comparable.

# Lane Following Benchmark {#sec:actual_benchmarking level=sec status=ready}

## General information
  The Benchmarking is set up in a way such that up to to point 6 the procedure stays the same no matter what you are Benchmarking. Starting at point 6 there will be slight changes in the processing and analysis of the data depending on the Benchmark. However these changes are very small and the main changes are within the metrics etc.
  The goal of every Benchmark is to produce a final pdf file that reports the results and compares them to another Benchmark of your choice. Ideally this benchmark it is compared to is the mean of all the Benchmarks ran all over the world of this type ran with the standard Duckietown software (for example the stable daffy commit of the containers `dt-core`, `dt-car-interface`, `dt-duckiebot-interface`, `dt-ros-commons`).
  However to be able to compare your Software with another one in any type of Benchmark, you first need to run at least 2 experiments. For each experiment there will be some recorded bags etc which then will be processed, analyzed and evaluated.
  The resulting evaluations of each experiment you run will then be again analyzed and for all the meaningful measurements, the means, medians and standard deviations are calculated. For each meaningful measurement the [coefficient of variation](https://www.researchgate.net/post/What_do_you_consider_a_good_standard_deviation) is calculated and if this value is below 1 you get a green light to compute the final Benchmarking report. This means that you have to run at least to experiments and then start running the notebook that calculates the variation of your computed results after each new experiment. So the amount of experiments that need to be run depend on the stability of your results. As soon as you get a green light of all the important results, compute the mean of all the results over all the experiments you ran (at least two).
  With this .yaml file including all the means, you are finally ready to run the comparison/analysis of your results. This will then generate a pdf report that analysis your solution as well as compares it to the results of another Benchmark you selected(can be any Benchmark ran of the same type). Based on the final report file you get at the end you can easily tell if your Software solution did improve the overall Performance or not and where your solution is better and where it is worse.

## Hardware Check (Ignore for now):
  First of all, for each Duckiebot involved,  run the hw_check you can find within the cloned behaviour-benchmarking repository. Therefor, follow the following steps:

  * Go into the folder hw_check by running:
    - `cd ~/behaviour-benchmarking/packages/hw_check`
  * Build the container by running:
    - `docker -H BOTNAME.local build --no-cache -t hw_check:v1 .`
  * Then run it by running:
    - `docker -H BOTNAME.local run -it --network host -v /data:/data -v /sys/firmware/devicetree:/devfs:ro hw_check:v1`

    Then follow the instructions within the terminal.

    TODO: at the moment Hat and Pi not detected! Figure out why.

  * When the Docker Container has finished, visit: `http://BOTNAME.local:8082/config` and download the .yaml file with your information in the name.
  * Place the .yaml file within the _data/BenchmarkXY_ folder of your behaviour_benchmarking repository.
  * Furthermore, set up your own [autolab fleet rooster](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_fleet_roster.html)


## Duckiebot preparation:
  To be able to record a rosbag on your Duckiebot please follow the steps below. This rosbag records all messages publisher to the for the specific Benchmark important nodes.
  The steps below prepare Terminal 4 of the four terminals mentioned below. Please note that the rosbag that will be recorded will automatically be saved on your USB drive.

  * Plug a USB drive (of at least 32 Gb) into your Duckiebot
  * Ssh into your Duckiebot by running:
    - `ssh AUTOBOT_NAME`
    * Within your Duckiebot, create a folder with the name `bag` by running the command:
      - `sudo mkdir /data/bag`
    * Use the command `lsblk` to see where your USB drive is located. Under name you should see sda1 or sdb1 and the size should be a bit less then the actual size of your drive (about 28.7 Gb for a 32 Gb drive)
    * Then mount the folder created above to your USB drive by running:
      - `sudo mount -t vfat /dev/sdb1 /data/bag/ -o uid=1000,gid=1000,umask=000`
    * Then exit your Duckiebot by pressing `Crtl + d`
  * Then start and enter a container on your Duckiebot by running:
    - `dts duckiebot demo --demo_name base --duckiebot_name AUTOBOT_NAME`
  * Then prepare the command to record a rosbag within that container:
    * `rosbag record -O /data/bagrec/BAGNAME_duckiebot.bag --duration 50 /AUTOBOT_NAME/line_detector_node/segment_list /AUTOBOT_NAME/lane_filter_node/lane_pose /rosout`

    Please note that if you are using Master19 you should subscribe to `/AUTOBOT_NAME/lane_controller_node/lane_pose` instead of `/AUTOBOT_NAME/lane_filter_node/lane_pose`



## Place your Duckiebot within the map:
  Place your Duckiebot in the beginning of the straight part that is _on the bottom of the loop_ relative to the origin of the coordinate system (see image below) of the loop on the outer line such that the Duckiebot drives around the the loop counter-clockwise.

  ToDo add image to where to place DB

## Prepare 4 terminals:
  Make sure that you carefully read through the steps and the `note` section below before starting up everything. It is important that you start recording the bags at roughly the same time and press `a` to start lane following or indefinite navigation straight after. Therefor first starting the diagnostic toolbox, then start recording both the bags and straight after start lanefollowing by pressing 'a'.

  * Terminal 1: Run the diagnostic toolbox on your Duckiebot:
      * `dts diagnostics run -G Name_BehBench_LF -d 70 --type duckiebot -H BOTNAME.local`

  * Terminal 2: Start the keyboard control on your Duckiebot:

      * `dts duckiebot keyboard_control BOTNAME`

      To start lane_following press 'a' on your keyboard
  * Terminal 3: Open a Docker container ros being pre-installed by running the command below **or** record a rosbag directly on your computer if you have the necessary setup installed:
    - `dts cli`

    Then within this container record a rosbag that subscribes everything published by the localization system by running:

    - `rosbag record -a --duration=50 -O BAGNAME_localization.bag`

  * Terminal 4: Run the command already prepared above to record a rosbag that subscribes to the needed topics.

  * Note:
    - If your Duckiebot crashes, you can stop lane following by pressing `s`, **but** please let the recordings of the bags as well as the Diagnostic Toolbox finish normally.
    - If your Duckiebot leaves the loop, make sure that you do **not** stop lanefollowing until the Duckiebot was out of sight of the watchtowers for at least 3 seconds **or** just go and grab the Duckiebot
      when it left the loop and take it completely out of sight manually, then stop the lanefollowing.
    - For the BAGNAME please use the following convention:
      - `Country_Autolab_LoopName_Date_GithubUserName_HWConfig_SWConfig`
    * Where:
      - `Country`: is the country you ran your benchmark (Ex. CH, USA, DE etc.)
      - `Autolab`: is the Autolab you ran your benchmark in (Ex. ETHZ etc.)
      - `LoopName`: is the name of the loop on which you recorded your benchmark, at the moment this should be `linus_loop`
      - `Date`: is the date on which you ran the benchmark in the format DDMMYEAR(Ex. 17022020)
      - `GithubUserName`: is your github username (Ex. duckietown)
      - `HWConfig`: is the hardware configuration of the Duckiebot you used (Ex. DB18, DB19 etc.)
      - `SWConfig`: is the software configuration used on the Duckiebot (Ex. Daffy, Master19 etc.)

## File gathering:
  After the rosbag recording as well as the Diagnostic Toolbox have finished you can stop the Duckiebot by pressing 's' on your keyboard.
  Then do the follwing steps:

  * Exit the container of Terminal 4 by pressing: `crt+d`
  * Ssh into your Duckiebot again by running:
    - `ssh AUTOBOT_NAME`
  * Within your Duckiebot unmount the folder by running:
    - `sudo umount /data/bag`
  * Then remove the USB drive from your Duckiebot and plug it into your local Computer. Copy the `BAGNAME_duckiebot.bag` that should be on your USB drive into the folder `bag` on your local computer.
  * Copy the recorded rosbag of the localization system from the Docker container onto your local computer into the`path_to_bag_folder` (should be simply `bag`) by running:
    - `sudo docker cp dts-cli-run-helper:/code/catkin_ws/src/dt-gui-tools/BAGNAME_localization.bag ~/path_to_bag_folder`
    or generally:
    - `sudo docker cp Docker_Container_Name:/place_within_container/where_bag_was_recorded/BAGNAME_localization.bag ~/path_to_bag_folder`
  * Make sure that both the bags are readable by opening the `bag` folder in a terminal and running:
    - `sudo chmod 777 BAGNAME_localization.bag`
    - `sudo chmod 777 BAGNAME_duckiebot.bag`
  * To get the information recorded by the diagnostic toolbox, visit [dashboard](https://dashboard.duckietown.org/) and login using your Duckietown token. Then navigate to _Diagnostics_ and in the drop down menue _Group_ select _Name_BehBench_LF_ and in the drop down menu _Time_ the corresponding time when you ran the Benchmark. After add the data by pressing onto the green plus and download the _.json_ file by pressing the Download log button.
  * Place the download .json file within you `bag` folder and rename it to `BAGNAME_diagnostics.json`.

  * To help the Duckietown community to gather the logs of the recorded bags, please create a folder, named BAGNAME, containing the two bag files as well as the .json file. Make zip of this folder and upload it to the bag folder found under [this link](https://drive.google.com/drive/folders/1pkjvPl8VyOj8K6jeUHXSE0XNPyVqgQDg?usp=sharing).


6. Processing the recorded bags:
  You need to know where your bag is. The folder containing it is referred as `path_to_bag_folder` in the command below. It is recommended to create new separate folders for each Benchmark (with date and/or sequence number). If you followed the instructions above, your bags are located in the folder `bag`. Example for `path_to_bag_folder` is /home/linus/bag.
  * Cd into the package `08-post-processing` found in your `behaviour-benchmarking` repository by running:
      * `cd behaviour-benchmarking/packages/08-post-processing`
  * Then build the repository by running:
      * `docker build -t duckietown/post-processing:v1 .`
  * Then run the post_processor for the rosbag of the localization system by running:
    * `docker run --name post_processor -it --rm -e INPUT_BAG_PATH=/data/BAGNAME_localization -e OUTPUT_BAG_PATH=/data/processed_BAGNAME_localization.bag -e ROS_MASTER_URI=http://192.168.1.97:11311 -v path_to_bag_folder:/data duckietown/post-processing:v1`

    This runs a slightly modified version of the original found [here](https://github.com/duckietown/duckietown-cslam/tree/master/08-post-processing).

    (Or run the original
    * `docker run --name post_processor -dit --rm -e INPUT_BAG_PATH=/data/BAGNAME_localization -e OUTPUT_BAG_PATH=/data/processed_BAGNAME_localization.bag -e ROS_MASTER_URI=http://YOUR_IP:11311 -v PATH_TO_BAG_FOLDER:/data duckietown/post-processor:daffy-amd64`)

    When the container stops, you should have a new bag called `processed_BAGNAME_localization.bag` as well as a new .yaml file called `BAGNAME_db_estimation.yaml` inside of your `path_to_bag_folder`. (This can take more than a minute, please be patient)
  * Remember from [Unit B-4 - Autolab map](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_map_making.html), that you created a map. Now is the time to remember on which fork you pushed it (the default is duckietown), and what name you gave to your map (for this Benchmark this should be `linus_loop`). The map file needs to be in the same folder as the rest of the maps. They are respectively the YOUR_FORK_NAME and YOUR_MAP_NAME arguments in the following command line. Please run the graph-optimizer by running:
     * `docker run --rm  -e  ATMSGS_BAG=/data/processed_BAGNAME_localization.bag -e OUTPUT_DIR=/data -e ROS_MASTER=YOUR_HOSTNAME -e ROS_MASTER_IP=YOUR_IP --name graph_optimizer -v path_to_bag_folder:/data -e DUCKIETOWN_WORLD_FORK=YOUR_FORK_NAME -e MAP_NAME=YOUR_MAP_NAME duckietown/cslam-graphoptimizer:daffy-amd64`

     This will generate at least one _.yaml_ file that will be stored in the folder `path_to_bag_folder`. If you followed the instructions and placed an April Tag with a different name than you Duckiebot on your localization standoff, you should find two _.yaml_ files. One will be named like your Duckiebot, and the other one like the name of the April Tag on you Duckiebot (Ex. autobot01.yaml). For the benchmarking we are only interested in the .yaml file that has the same name as the April Tag on top of your Duckiebot has.

  * For the rosbag recorded on the Duckiebot, run analyze-rosbag by:
    * cd into the `analyze_rosbag` folder found in behaviour-benchmarking repository by running `cd behaviour-benchmarking/packages/analyze_rosbag`
    * Build the repository by running:
          * `dts devel build -f --arch amd64`
    * Then run it with:
          * `docker run -v path_to_bag_folder:/data -e DUCKIEBOT=AUTOBOT_NAME -e BAGNAME=BAGNAME_duckiebot -it --rm duckietown/behaviour-benchmarking:v1-amd64`
    * This will create five `.json`files within the `bag`folder that will be used for the Benchmarking later. The _.json_ files are named: `BAGNAME_duckiebot_constance.json`, `BAGNAME_duckiebot_lane_pose.json`,`BAGNAME_duckiebot_node_info.json`,`BAGNAME_duckiebot_segment_count.json`, `BAGNAME_duckiebot_latencies.json`
    * Make sure that those files are readable by opening the `bag` folder in a terminal and running:
        * `sudo chmod 777 FILENAME.json`

ToDo: explain the results that are  given after the processing, like explain what the latency exactly is etc. Specially about the .json files


7. Result analysis preparation:
  * Place the .yaml file created by the graphoptimizer with the name of the April Tag that is on top of your Duckiebot into the _data/BenchmarkXY/yaml/graph_optimizer_ folder (please note that it is important that you take the correct .yaml file as the one named after your actual Duckiebot should **not** be placed within the mentioned folder). Then place the .yaml file created by the post_processor called `BAGNAME_db_estimation.yaml` into the _data/BenchmarkXY/yaml/post_processor_ folder. Also place all the .json files (the one downloaded from the dashboard as well as the 5 created by the analyze_rosbag container) into the  _data/BenchmarkXY/json_ folder of your behaviour_benchmarking repository.
  * Note that XY stands for a number, so for your first Benchmark name the folder _Benchmark01_.
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
    5. Navigate to the notebooks

8. Result computation:
  * First open the notebook called: `94-analyse_dashboard.ipynb` and follow the instructions there. This will result in two .yaml files, one called `BAGNAME_software_information.yaml` and the other one called `BAGNAME_eng_perf_data_all.yaml` placed within the folder `behaviour_benchmarking/data/BenchmarkXY/out`.

      * The `software_information` file includes informations of all the containers like: container name, image name and tag, the base image of the container, the architecture of the container, the branch etc. as well as the constants that were  set within the Duckiebot for example the gain, the trim factor etc.
          These things do not change within the same Benchmark, this means for all the tests you are running with the specific software version all this information remains the same. This yaml file is used in the end for the final Benchmark report.
      * The `eng_perf_data_all` file on the other hand includes all kind of engineering data analysis like the update frequency of the different nodes, the number of segments detected over time, the latency up to and including the `detector_node`, as well as the CPU usage, the Memory usage and the NThreads used by each container. This data changes (at least slightly) between two different tests of the same Benchmark which is why the mean of this data of all the tests ran for one Benchmark is calculated later.

  * Then open the notebook called `95-Trajectory-statistics` and follow the instructions there. This will result in a .yaml file called BAGNAME__benchmark_results_test_XY.yaml where XY is the number of the test run.

### Metrics
Arithmetic mean:
$$\bar{x}=\frac{1}{n} \sum_{i=1}^n x_i $$

Standard deviation:
$$s=\sqrt{\frac{1}{N}\sum_{i=1}^N(x_i-\bar{x})^2}$$

Note: the localization system that measures the ground truth, measures the position of the Apriltag placed on the localization standoff of your Duckiebot. This means that if this Apriltag is not placed very accurately, your results will be false.

Engineering score: Update frequency, latency, cpu usage etc
Behaviour score: offset (distance and angle), calculated d and phi compared to ground truth, distance travelled, duration of Benchmark

ToDo: Spit out type of tile where Benchmark was stopped in case of an early termination

### Termination Criteria

The Benchmark is officially terminated after the 50 seconds are up. However when the Duckiebot is out of sight for more then 3 seconds or if the Duckiebot takes more then 30 seconds
to get across 1 tile the Benchmark will be terminated early. This will be taken into account into the final Benchmark score.
An early termination will not be considered as a failiure but will just lead to a lower score.

# For distance keeping:
interesting nodes:
/autobot01/vehicle_avoidance_control_node/car_cmd                            171 msgs @   5.3 Hz : duckietown_msgs/Twist2DStamped  
             /autobot01/vehicle_avoidance_control_node/switch                               3 msgs @   0.1 Hz : duckietown_msgs/BoolStamped     
             /autobot01/vehicle_avoidance_control_node/vehicle_detected                    40 msgs @   2.1 Hz : duckietown_msgs/BoolStamped     
             /autobot01/vehicle_detection_node/detection                                   41 msgs @   2.1 Hz : duckietown_msgs/BoolStamped     
             /autobot01/vehicle_detection_node/detection_time                              40 msgs @   2.0 Hz : std_msgs/Float32                
             /autobot01/vehicle_detection_node/switch                                       3 msgs @   0.1 Hz : duckietown_msgs/BoolStamped     
             /autobot01/vehicle_filter_node/switch                                          3 msgs @   0.1 Hz : duckietown_msgs/BoolStamped

# Future work
Improve the out_of_sight condition such that the Watchtowers can still see the Duckiebot but it counts as out of sight if he is on a tile which is not part of the loop for longer then 3 seconds.
save data online automatically (not up to user)
set up remaining Benchmarks
