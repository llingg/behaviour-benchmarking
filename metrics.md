# Benchmarks and their Metrics

## Lane following
### Big Benchmark
Metrics:
* distance it got in a given time -> \#loops: l
* time lane following ran before stop: t (Max = 50sec)
* std of position and angle in straight sections to middle of the lane: std_pos_straight, std_pos_straight
* std of position and angle in curved sections to middle of the lane: std_pos_curve, std_angle_curve
* std of pose estimation (pose and angle) compared to ground truth: std_pose_est

Termination criteria:
* Duckiebot out of sight for more than 3 seconds
* Duckiebot takes more than 10 seconds to get across 1 tile (case for example when it crashed)

Score:
* 1/l * 0.1 + (std_pos_straight+std_pos_straight)* 0.2 + (std_pos_curve+std_angle_curve)* 0.2 + 1/std_pose_est * 0.3 + abs(50-t)

* Based on ground truth (given by Watchtowers) calculate standard deviation of the lane offset and the angle using the following formula:
Standard deviation:
  $$s=\sqrt{\frac{1}{N}\sum_{i=1}^N(x_i-\bar{x})^2}$$
  where $\bar{x}$ is the middle of the lane and $x_i$ is the actual lane position measured by the Watchtowers. The same applies for the heading angle deviation where one must be careful because TODO see notebook
* Difference between calculated position (offset, angle) and ground truth (measured by Watchtowers) -> differ between straights and curves:
  $$s=\sqrt{\frac{1}{N}\sum_{i=1}^N(x_{DB}-x_{WT})^2}$$
* Time needed per tile
* Engineering data:
  - CPU usage
  - Delay, Update freq, Lag of different components
* Robustness
  - Time needed to get back into lane after being out of lane
  - How well does it work when for example white lines covered
### Small Benchmark
* Filtering:
  - Turn DB on spot and compare calculated lane position with ground truth measured by Duckiebot

## Vehicle Detection
### Big Benchmark
Metrics:
* Max dist object_detected
* Relative pose estimation

Engineering data
* Lag
* update frequency
* CPu usage

* Distance at which backplate is detected and accuracy of relative pose calculation (Let DB publish calculated distance to DB in front with the corresponding timestamps, then simply run the graphoptimizer as well as extract the distance and timestamp data of the recorded bag, load it in a Notebook, calculate relative distance btw Duckiebots from Data of Watchtowers and compare to distance calculated by DB):
  * Place one "dummy" Duckiebot in the end of a straight line.
  * Set gain of actual Duckiebot to 0.5
  * Start indefinite_navigation -> record bag
  * From the bag using ground truth of Watchtowers, get distance at which 2nd Duckiebot is seen, compare calculated distance to Duckiebot in front compared to ground truth of Watchtowers.
  * Repeat the same experiment in a curve
  * calculate the standard deviation:
    $$s=\sqrt{\frac{1}{N}\sum_{i=1}^N(d_{DB}-d_{WT})^2}$$

### Small Benchmark
* The controller of the following at a certain distance the DB in front (in case there is one):
  - Dummy DB with gain = 0.5 does normal LF and test DB has gain = 1 and does lf behind it. Measure the relative distance over time and calculate standard deviation of goal distance it should keep.


## Red line detection
### Big Benchmark
Perception:

* detect the red line → lag and update frequency, reliability (how often and at what distances is it detected

* determine relative pose → compare calculated relative pose with actual relative pose (detected by watchtower)

Control:

* Distance and angle of DB to red line after he stopped

### Small Benchmark


## Intersection navigation
### Big Benchmark
Perception:

* LED detection (Blob detection and Pattern/Color detection):

* Reliability depending on type of LED (update frequency and lag)

* Reliability depending on Light condition  (update frequency and lag)

* April Tag detection (Image to April Tag and April Tag to Meaning):

* Reliability depending on type/pattern/color of LED  (update frequency and lag)

* Reliability depending on Light condition  (update frequency and lag)

Control:

* Action depending on detection

### Small Benchmark

## Intersection coordination
### Big Benchmark
* Average time of DB at intersection

* Number of crashes

### Small Benchmark

## City global navigation
### Big Benchmark


### Small Benchmark


## Overtaking
### Big Benchmark
* Time needed, \#crashes

### Small Benchmark
