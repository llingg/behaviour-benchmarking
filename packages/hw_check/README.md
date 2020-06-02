# duckiebot_hw_checks

This package is part of any kind of Behaviour Benchmark. It has to be run after all the Hardware is set-up.

Build it by running:
- ´docker -H BOTNAME.local build --no-cache -t hw_check:v1 .´
Then run it with:
- ´docker -H BOTNAME.local run -it --network host -v /data:/data -v /sys/firmware/devicetree:/devfs:ro hw_check:v1´

This will ask the user a couple of questions that need to be answered carefully, some further instructions can be found [here](https://gitlab.com/llingg/behaviour_benchmarking/-/blob/master/hw-checklist.md). 
The results respectively the final yaml file that is stored when the container has finished can be downloaded from: ´http://BOTNAME.local:8082/config ´

## Future work
At the moment Hat and Pi not detected -> Figure out why
