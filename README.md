# d2dtracker_drone_detector
A ROS package with nodes that implements drone detection using neural netowrk and depth maps.

![D2DTracker System Architecture](images/d2dtracker_system_architecture.png "D2DTracker System Architecture")

**NOTE**

**This repository is part of the D2DTracker work which is submitted to the IROS 2023 conference. The code will be availble once the paper is accepted.**

# Installation
* The `main` branch of this repository contains the ROS 2 `humble` version of this package.
* This package is installed as part of the `d2dtracker` development environment, see installation instructions in the [d2dtracker_sim](https://github.com/mzahana/d2dtracker_sim) package.

# Run
* Make sure this package is inside the ROS 2 workspace
* Make sure that you build the ROS 2 workspace, and source `install/setup.bash`
* To run the detection system, run the following launch file,
    ```bash
    ros2 launch d2dtracker_drone_detector detection.launch.py
    ```

**NOTES**
* Make sure that you provide the correct depth image topic in [here](https://github.com/mzahana/d2dtracker_drone_detector/blob/366cf6440327db84f493fca2337a3b551edffeb2/launch/detection.launch.py#L28), and the camera info topic [here](https://github.com/mzahana/d2dtracker_drone_detector/blob/366cf6440327db84f493fca2337a3b551edffeb2/launch/detection.launch.py#L33)
* There should be a valid static transformation between the drone frame and the camera frame. For an example, see [here](https://github.com/mzahana/d2dtracker_sim/blob/5ea454e95fd292ab16cb3d28c50bb2182572ad52/launch/interceptor.launch.py#L94). This is required to compute the position of the detected drone in the interceptor's localization frame, which can be sent to a Kalman filter in a later stage.
* You can configure the depth-based detection parameters in [here](https://github.com/mzahana/d2dtracker_drone_detector/blob/366cf6440327db84f493fca2337a3b551edffeb2/config/detection_param.yaml)
* Make sure that you build your workspace after any modifications, using `colcon build`