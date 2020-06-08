# Light LF

This package gives the user an easy environment to add their contribution to dt-core in order to Benchmark any kind of Behaviour.

If nothing is added respectively changed, this package serves as the daffy version of dt-core.

Build this package by running:
* `docker -H autobot01.local build --no-cache -t light_lf:v1 .`
Then run it with:
* `docker -H BOTNAME.local run -it  --name behaviour_benchmarking --rm -v /data:/data --privileged --network=host light_lf:v1`

To start lane following respectively indefinite navigation press ´a´ in the keyboard control which is started using the command:
* `dts duckiebot keyboard_control BOTNAME`

You can easely change between lane following and indefinite navigation by uncommenting line 3 in the light_lf.launch file found in light_lf/packages/light_lf/launch folder and commenting out line 2.

To add your contribution simply add the packages you worked on/ modified into the folder ´packages/light_lf/packages´. Make sure the names of the folder is correct, this means the folder have the same name as the different packages found in [dt-core](https://github.com/duckietown/dt-core/tree/daffy/packages).
Your package will then automatically replace the original one when building and running with the commands found above.