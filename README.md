# leddar_ros2

This is a first attempt at migrating from ros1 to ros2, to provide a ros2 package to get PointCloud message from leddar sensors. **It has not been trough complete and extensive testing procedure.**

****Tested hardware and software:**** This package has been tested with a Leddar Pixell with ros2/eloquent (installed from debian) on Ubuntu 18.04 with python 3.6.9 version.

## Dependencies

The only ros node that publishes PointCloud2 messages is written in python and uses rclpy from ros and leddarpy from [LeddarSDK](https://https://github.com/leddartech/LeddarSDK)

You need to first to build and install **LeddarSDK leddar python module** from the documentation of the LeddarSDK here : [https://github.com/leddartech/LeddarSDK/blob/master/README.md](https://https://github.com/leddartech/LeddarSDK/blob/master/README.md)

### Build the node

The easiest would be to install [ros2 from via debian package](https://https://docs.ros.org/en/eloquent/Installation/Linux-Install-Debians.html) (link provided for ros2/eloquent on Ubuntu 18.04 (the tested setup)). In short:

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-eloquent-desktop
```

Once installed, you may [create a workspace](https://https://docs.ros.org/en/eloquent/Tutorials/Workspace/Creating-A-Workspace.html) as described in ROS documentation.

Do not forget to [source your ros2 environment](https://https://docs.ros.org/en/eloquent/Tutorials/Configuring-ROS2-Environment.html#configros2) to have access to ros2 command-line tools, as an example for eloquent:`source /opt/ros/eloquent/setup.bash`

Then, you can [build the package using colcon](https://https://docs.ros.org/en/eloquent/Tutorials/Colcon-Tutorial.html): `colcon build --symlink-install --packages-select leddar_ros2`

You may source your [setup.bash file](https://https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html#source-the-setup-file), to use the newly build package : `. install/setup.bash`

Finally, you should be ready to use leddar_ros2 package.

### Launch leddar_sensor

To run the leddar_sensor node which publishes PointCloud2 messages from the sensor you may simply use the ros2 run command as in the following example:

* To run the node: `ros2 run leddar_ros2 leddar_sensor`
* To visualize the data in rviz2: `ros2 run rviz2 rviz2`

  * Once rviz is open click on the add button -> Create vizualization -> By topic and select /scan_cloud or load the leddar_ros2/config/leddar_rviz.rviz configuration file.

**Node: The default value for the arguments are configured for a pixell (Ethernet) sensor.** The `leddar_sensor` node should allow connection to all sensor supported within LeddarSDK. To do so it uses the same parameters/arguments as the connect method of the Device class of leddarpy module [here](https://https://sdk.leddartech.com/v4.3/#/Python?id=device-class).

You may launch `leddar_sensor` node with parameters depending on the sensor that you have with the following command:

* For example, with a pixell sensor :`ros2 run leddar_ros2 leddar_sensor --ros-args -p param1:=192.168.0.2 -p device_type:=Ethernet`
* Or an M16 sensor with Usb communication`ros2 run leddar_ros2 leddar_sensor --ros-args -p param1:=AL26026 -p device_type:=Usb`
* See below a more detailed description of the parameters

#### Description of leddar_sensor parameters:

param1: parameter of type (string)

* Its value would depend on the device_type:
  * For [Serial] -> serial port com
  * For [USB] -> serial number
  * For [SPI-FTDI] -> FTDI cable ID (use get_devices("SpiFTDI"))
  * For [Ethernet] -> IP address
* device_type: parameter of type (string)
  * `Serial`, `SpiFTDI`, `Ethernet`or`Usb`
* param3: parameter of type (optional, int)
  * Its value would depend on the device_type
    * For [Serial] -> Modbus address (default 1)
    * For [Ethernet] -> port (default 48630)
* param4: parameter of type (optional, int)
  * Its value would depend on the device_type
    * For [Serial] -> baudrate (default 115200)
    * For [Ethernet] -> Communication timeout
