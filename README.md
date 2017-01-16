## Witham Wharf transforms

The Witham Wharf transforms package is meant to complement the Witham Wharf long-term [dataset](https://lcas.lincoln.ac.uk/owncloud/shared/datasets/).
The package aims to provide a complete **tf** tree, which would allow to transform the data provided in the rosbags to a global coordinate (**map**) frame.
To test the package, you need to 

1. Make sure that ROS will use the simulated time: **rosparam set use_sim_time true**
1. Launch the transformer and the map: **roslaunch ww_transforms ww_transforms.launch**
1. Play one of the rosbags of the WW dataset, e.g: 

