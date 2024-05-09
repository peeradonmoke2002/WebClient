# WebClient

This is a ROS (Robot Operating System) package designed to support a web application.

## Description

This package provides the necessary interfaces and tools to integrate a web application with a ROS-based system. It allows the web application to interact with the ROS system, send commands, and receive data.

## Usage
To build the ROS workspace and launch the WebClient, run the following commands:

```bash
cd ~/{ROS_WORKSPACE} && catkin_make
roslaunch webclient start_all.launch
```

Try rosservice to start turtlebot simulation 
```bash
rosservice call /gmapping_amcl_switcher "launch_command: 'SIM'" 
```

let's control robot in web-application

## File use for Develop

- waypoint_server.py: waypoint ros server
- wifi_network.py: use for monitor wifi ros parameter

<!-- ## Installation

Provide instructions on how to install this package.

## Usage

Provide instructions on how to use this package.

## Contributing

Provide instructions on how to contribute to this project.

## License

Provide information about the license. -->