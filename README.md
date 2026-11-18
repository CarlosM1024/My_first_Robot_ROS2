<div align="center">

# 🤖 `my_robot`: First Robot Template in ROS 2

**Step-by-step guide to building a differential-drive robot URDF model, adding collision and inertial properties, visualizing in RViz2 & GazeboSim, and driving it in ROS 2.**

[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Simulator](https://img.shields.io/badge/Simulation-RViz2_%7C_GazeboSim-0A506B?style=for-the-badge)](https://gazebosim.org/)
[![Credit](https://img.shields.io/badge/Tutorial_Credit-Rob%C3%B3tica_con_Python_(cmorales)-informational?style=for-the-badge)](https://www.youtube.com/@roboticaconpython)

</div>

> [!NOTE]
> **Credit & Attribution:** This guide and robot template are based on the tutorial series by **Carlos Morales (*Robótica con Python*)**, adapted with clean formatting and expanded notes for study and experimentation.

---

## 📑 Table of Contents

- [1. Add the Repository to Your Workspace](#1-add-the-repository-to-your-workspace)
- [2. Explore the File Structure](#2-explore-the-file-structure)
- [3. Model the Basic Robot Structure (`robot_base.urdf`)](#3-model-the-basic-robot-structure-robot_baseurdf)
- [4. Add Inertial Parameters and Collision Geometry](#4-add-inertial-parameters-and-collision-geometry)
- [5. Visualizing the Model in GazeboSim](#5-visualizing-the-model-in-gazebosim)
- [6. Next Steps: Motion Control & Teleoperation](#6-next-steps-motion-control--teleoperation)

---

## 1. Add the Repository to Your Workspace

In a terminal, open the `src` directory of your ROS 2 workspace and clone this repository:

```bash
git clone https://github.com/CarlosM1024/My_first_Robot_ROS2.git
```

Build the workspace:

```Bash
cd <path_to_your_workspace>
colcon build --symlink-install
```

Now run the launch file to verify that everything works properly:

```Bash
source install/setup.bash
ros2 launch my_robot view.launch.py
```

The launch file opens an **RViz2** window and starts a node that publishes the URDF model description over the `/robot_description` topic. However, you still need to configure **RViz2** so the model displays properly:

- In the **Displays** panel, add the **RobotModel** plugin. Under **RobotModel** properties, verify that **Description Source** is set to **Topic**, then select /`robot_description` under **Description Topic**. A small red box should appear in the RViz2 viewport.

- If the model color or transforms do not display as expected, set *Fixed Frame* to `base_link` under **Global Options**.

- Save this configuration from the **File** menu so you do not have to repeat the setup every time you run the launch file.

This confirms that the base template works on your machine. Next, you will turn this base model into your first ROS 2 robot.

## 2. Explore the File Structure

Open a VS Code window inside the repository folder: 

```Bash
code src/my_robot
```

The directory and file layout in the repository is organized as follows:

- Standard files and folders located at the root of the repository (the same layout found in `ament_python` ROS 2 packages): `package.xml`, `setup.cfg`, `setup.py`, and the `resource/`, `my_robot/`, and `test/` directories.

- A `description/` folder to store the `.urdf` and `.xacro` files that define the robot model.

- A `launch/` folder to store launch files, allowing you to start multiple nodes from a single terminal with custom configurations and parameters.

This basic structure will be expanded later as you add robot capabilities and run simulations in **GazeboSim**.

## 3. Model the Basic Robot Structure (`robot_base.urdf`)

**URDF (Universal Robot Description Format)** is the XML format used in ROS to describe a robot's links and joints.

Initially, the file defines a robot consisting of a single `base_link` visually shaped as a `40 x 25 x 15 cm` red box centered at the coordinate origin:   

```XML
<?xml version="1.0"?>
<robot>
    <link name="base_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <box size="0.4 0.25 0.15" />
            </geometry>
            <material name="base_link_material">
                <color rgba="0.85 0.0 0.0 1.0" />
            </material>
        </visual>
    </link>
</robot>
```

Start by adjusting the box position so it is lifted off the ground:

```XML
<origin xyz="0 0 0.125" rpy="0 0 0" />
```

Next, add a left wheel by creating a `continuous` joint and a new `link` inside the `<robot>` tag, using a cylinder geometry to represent the wheel:  

```XML
    <joint name="left_wheel_joint" type="continuous">
        <origin xyz="-0.1 0.15 0.05" rpy="-1.570796 0 0" />
        <axis xyz="0 0 1" />
        <parent link="base_link" />
        <child link="left_wheel" />
    </joint>
    <link name="left_wheel">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <cylinder length="0.04" radius="0.05" />
            </geometry>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1.0" />
            </material>
        </visual>
    </link>
```

When you run the launch file again, you will notice the wheel does not appear in the expected place and RViz2 reports a **RobotModel** error because there is no joint state published yet for `left_wheel_joint`.

To fix this, run a node that publishes joint positions on `/joint_states` (or /`joint_status` as noted in the tutorial). In a new terminal, run:

```Bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

Since you will want this available every time you edit the URDF and run the launch file, add `joint_state_publisher_gui` to `view.launch.py.`

Inside `generate_launch_description()`, define the node:

```Python
...
joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
...
```

And include `joint_state_publisher_gui_node` in the returned `LaunchDescription` list:

```Python
return LaunchDescription([
    ...
    joint_state_publisher_gui_node,
    ...
])
```

Now add the right wheel:  

```XML
    <joint name="right_wheel_joint" type="continuous">
        <origin xyz="-0.1 -0.15 0.05" rpy="-1.570796 0 0" />
        <axis xyz="0 0 1" />
        <parent link="base_link" />
        <child link="right_wheel" />
    </joint>
    <link name="right_wheel">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <cylinder length="0.04" radius="0.05" />
            </geometry>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1.0" />
            </material>
        </visual>
    </link>
```

Finally, add a `fixed` joint and a small sphere to act as a caster/support wheel:

```XML
    <joint name="connection_link" type="fixed">
        <origin xyz="0.15 0 0.05" rpy="0 0 0" />
        <axis xyz="0 0 1" />
        <parent link="base_link" />
        <child link="caster_wheel" />
    </joint>
    <link name="caster_wheel">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <sphere radius="0.05" />
            </geometry>
            <material name="black">
                <color rgba="0 0 0 1.0" />
            </material>
        </visual>
    </link>
```

## 4. Add Inertial Parameters and Collision Geometry

Your first robot model now has visual geometry. However, for it to behave properly in a physics simulation, you need to define **collision** geometry and **inertial** parameters for each link.

Keep the following guidelines in mind:

- **Collision geometry:** Defines the physical boundaries of the robot for collision detection. Using simple shapes reduces computational load in physics engines. For simple models like this one, you can mirror the ``<visual>`` geometry inside ``<collision>`` tags; for detailed 3D meshes, simpler collision meshes are recommended.

- **Inertial parameters:** Used by the physics engine to compute dynamics (forces and accelerations). Accurate inertia matrices are critical for stable and realistic simulation behavior. They can be exported from CAD software or approximated analytically using standard formulas for boxes, cylinders, and spheres.

For this first robot model, copy the visual geometry into ``<collision>`` blocks for each link and add the following ``<inertial>`` blocks:

```XML
    <link name="base_link">
        ...
        <inertial>
            <origin xyz="0 0 0.125" rpy="0 0 0" />
            <inertia iyy="0.016755" ixz="0.0" izz="0.016755" iyz="0.0" ixx="0.016755" ixy="0.0" />
            <mass value="4.2" />
        </inertial>
        ...
    </link>
    ...
    <link name="left_wheel">
        <inertial>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <inertia iyy="0.002788" ixz="0.0" izz="0.005147" iyz="0.0" ixx="0.002788" ixy="0.0" />
            <mass value="0.8" />
        </inertial>
    </link>
    ...
    <link name="right_wheel">
        <inertial>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <inertia iyy="0.002788" ixz="0.0" izz="0.005147" iyz="0.0" ixx="0.002788" ixy="0.0" />
            <mass value="0.8" />
        </inertial>
    </link>
    ...
    <link name="caster_wheel">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0" />
            <inertia iyy="0.001047" ixz="0.0" izz="0.001047" iyz="0.0" ixx="0.001047" ixy="0.0" />
            <mass value="0.5" />
        </inertial>
    </link>
    ...
```

With this complete, your base robot model is ready.

## 5. Visualizing the Model in GazeboSim

Even before adding a motion controller, you can spawn the model inside a **GazeboSim** simulation.In a terminal, start GazeboSim and choose a world:

```Bash
gz sim
```

In another terminal, spawn the robot using the URDF published on `/robot_description`:

```Bash
ros2 run ros_gz_sim create -topic /robot_description
```

If no specific spawn coordinates are passed, the robot will appear at the origin `(0, 0, 0)` of the simulation world.

[!WARNING]This spawning workflow applies to URDF models built from basic geometric primitives. Models relying on detailed external mesh files require a different setup/procedure.

## 6. Next Steps: Motion Control & Teleoperation

Once the base URDF model is built and visualized, the workflow continues with additional modules and tutorials to give the robot movement and control capabilities:

- Additional Tutorial Reference: [ROS 2 Robot Motion & Control Tutorial](https://www.youtube.com/watch?v=zIkWGv5pKLY)

### `my_robot_controller`

A basic open-loop controller node that listens for velocity commands on `/cmd_vel` and publishes estimated robot pose/odometry on `/odom`.

- Detailed Tutorial: [my_robot_controller Guide](https://www.youtube.com/watch?v=3Q9U6D1uC2U)

To publish velocity commands (`geometry_msgs/msg/Twist`) from the terminal at `10 Hz` (forward linear velocity `0.2 m/s` and angular yaw rate `0.2 rad/s`):

```Bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

### `joy_velocity_publisher`

Translates joystick inputs into `/cmd_vel` velocity commands so you can teleoperate the robot manually.

- Detailed Tutorial: [joy_velocity_publisher GuideQuick Verification Commands](https://www.youtube.com/watch?v=mP-rBK-ST-8)

While testing your controller or joystick teleoperation node, you can inspect active topics and verify that `/cmd_vel` and `/odom`are flowing as expected:

```Bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /odom
```

## References

- **Reference Repository:** [cmoralesd/my_package](https://github.com/cmoralesd/my_package)

- **YouTube Channel:** [Robótica con Python (Carlos Morales)](https://www.youtube.com/@roboticaconpython) and [Articulated Robotics (Josh Newans)](https://www.youtube.com/@ArticulatedRobotics)

- **Official Documentation:** [ROS 2 Jazzy Jalisco Documentation](https://docs.ros.org/en/jazzy/About-ROS.html)

- **Build Tool:** [colcon Documentation](https://colcon.readthedocs.io/en/released/user/quick-start.html)
