# behaviour_benchmarking

##Requiries


<div class='requirements' markdown='1'>
  Requires: [Duckiebot (correctly assembled, initialized and calibrated)](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/building_duckiebot_c0.html)
  
  Requires: [Localization standoff](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_autobot_specs.html)
  
  Requires: [Autobot](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_autobot_specs.html)
  
  Requires: [2 straight and 4 curved tiles](https://docs.duckietown.org/daffy/opmanual_duckietown/out/dt_ops_appearance_specifications.html) 
  
  Requires: [At least 4 Watchtowers in WT19B configuration](https://docs.duckietown.org/daffy/opmanual_autolab/out/watchtower_hardware.html)
  
  Requires: [X Ground April tags](https://github.com/duckietown/docs-resources_autolab/blob/daffy/AprilTags/AprilTags_localization_ID300-399.pdf)
  
  Results: Lane Following Benchmarking
</div>

<minitoc/>

First assemble the tiles in a way sucht that a 2x3 closed loop as seen below is formed.
![linus_loop](/media/linus_loop.jpg)
Further place the Watchtowers around the loop susch that each part of it is seen by at least one Watchtower. To get really accurate results it is recommended to place them as seen below
![WT position](/media/TODO)
Then place the Ground April Tags roughly as seen in the images above and measure the exact position.
Clone the [Duckietown-world repository](https://github.com/duckietown/duckietown-world) and follow the instructions found [here](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_map_making.html)
to create your own map. Or simply add the linus_loop folder into the $visualization/maps folder of your duckietown-world repository and the [linus_loop][TODO link to yaml file] into ToDo.

After follow the instructions found [here](https://docs.duckietown.org/daffy/opmanual_autolab/out/localization_apriltags_specs.html) to add them to the map.
Then set up the [offline localization](https://docs.duckietown.org/daffy/opmanual_autolab/out/localization_demo.html).


You can use the following command to record bag files

    $ rosbag record -a -o BAG_NAME.bag

