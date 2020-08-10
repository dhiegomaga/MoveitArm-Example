# MoveitArm-Example

Example of fixing the auto generated code given by moveit setup assistant (ROS Melodic) to work with Gazebo simulator and the real robot.

The chosen arm was the [Edo Comau](https://github.com/dhiegomaga/eDO_description)

### Test Gazebo simulation using MoveIt

```bash
roslaunch edo_moveit_config gazebo.launch
```

```bash
roslaunch edo_moveit_config moveit_planning_execution.launch
```

## Modifications

The auto generated files do not work, therefore this repository's commits describe how to fix the files and hopefully get it working.

I will _probably_ add an example of how to create a controller for the real arm as well.

## Issues Encountered

### Robot Collapsing

One issue I encountered was that the arm was **collapsing to the center of the map** as soon as I tried controlling it using MoveIt. I also got the error from the MoveIt package:

```bash
Controller failed with error code GOAL_TOLERANCE_VIOLATED
```

But this error is not caused by MoveIt, since I was also able to reproduce the problem without MoveIt, just using an outside controller.

For that I tried controlling it using the graphical interface for control, [rqt_joint_trajectory_controller](http://wiki.ros.org/rqt_joint_trajectory_controller), and the collapse continued to occur. It turned out that I needed to change the PID control values and the inertia/mass of each link. [This](https://answers.gazebosim.org//question/4102/my-robot-blows-up-when-i-launch-the-controllers-update-2/) hinted me to this solution.

## Credits

Largely inspired by [this repository](https://github.com/kkumpa/ros-robotic-arm). However, MoveIt did not work with it, and I hypothesized it was because the arm in question did not possess 6 DoF (only 4), and hence the IK solver was not supposed to find a solution anyway.
