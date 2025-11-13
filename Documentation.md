# Procedure to Replicate Results

## Requirements
* ROS2 Humble Hawksbill environment
* `franka_ros2` package built into a workspace

## Changes to be made to franka package
* First we need to add our custom controller into the mix, so we navigate to `franka_example_controllers` using:
```
cd franka_ros2_ws/src/franka_example_controllers/src
```

* Then we add our `custom_controller.cpp` and `custom_controller.hpp` files to the `src` and `include` folders respectively.

* Next we modify `CMakelists.txt` file by adding the following line under `add_library` (you will notice other controller files as well):
```
src/custom_controller.cpp
```

* Once that is done, we move on to the `franka_example_controllers.xml` file and add the following bit under `<library path="franka_example_controllers">:` (you will notice other controller files as well)::
```
<class name="franka_example_controllers/CustomController"
        type="franka_example_controllers::CustomController" base_class_type="controller_interface::ControllerInterface">
    <description>
        The scots vcz example controller is a custom EE trajectory tracker.
    </description>
</class>
```

NOTE: `CustomController` refers to the main class inside `custom_controller.cpp`


### For Simulation using gazebo and rviz
Now we move on to actually using the custom controller that we have just created. For that we would need to create a launch file that calls our controller and visualizes the robot in rviz and gazebo!

* Navigate to `franka_gazebo_bringup` using:
```
cd franka_ros2_ws/src/franka_gazebo/franka_gazebo_bringup
```

* Here you will see two folders: `config` and `launch`. We navigate to `launch` and create our launch file here as `gazebo_custom_controller_example.launch.py`

* Next we navigate to the `config` folder which contains the `franka_gazebo_controllers.yaml` file. Then we add the following lines under `controller_manager: -> ros__parameters:` :
```
    custom_example_controller:
      type: franka_example_controllers/CustomController
```

And you're all set to launch your custom controller using:
```
cd franka_ros2_ws/
source install/setup.sh
source /opt/ros/humble/setup.sh
ros2 launch franka_gazebo_bringup gazebo_custom_controller_example.launch.py load_gripper:=true franka_hand:='franka_hand'
```

### For Hardware Implementation
--pending--
