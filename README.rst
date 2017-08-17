===========================================
A Ros package for using a Swarm of Spheros
===========================================

This package allows the control of a swarm of spheros through ros topics. It publishes and subscrbes to the same topics as the base Sphero package. However each topic's data type only has two fields: the name of the sphero Name, and the data field which is the same type of what the topic expects in the sphero. ie. odom publishes a topic with a name field and a odometry field.

It works by starting a new node for each sphero in a new namespace under the swarm nodes namespace, a bit like roslaunch. Rqt_console should show a tree like structure. The node then forwards relevent commands to each sphero, and combines sensor data into single topics.

INSTALL
---------

..code-block:: terminal

    cd catkin_ws/src

    git clone https://gitlab.com/hcmi/sphero_swarm.git

    cd ..
    catkin_make install


TOOLS
------

Each tool can be started with rusrun, or through the provided launch files.


**sphero_swarm_manager**
  A gui tool that allows the user to dynamicallly add and remove spheros

**sphero_swarm_dashboard**
  A gui tool that allows sphero color and heading customization.

**keyboard_teleop**
  A gui tool that allows teleoperation of a single sphero at a time. To use select ther Sphero you wish to control, and then click the white text box.


STARTING THE SWARM
-------------------

The swarm can be started by running ``sphero_swarm_node.py`` or by calling `roslaunch sphero_swarm swarm.launch``. The first will not auto connect to any spheros, and all spheros will have to be manually added through the dash board.
The launch file will attempt to connect to the spheros listed in ``launch/param/swarm.yaml`` The yaml file defines a dictionary from names to addresses, and is loaded as a paramter in the launch file. Only seven of the listed spheros will be connected too.

CONNECTING MORE THEN 7
----------------------

Bluetooth devces have a limit of 7 connections each. In order to connect to more than 7 spheros you will need more bluetooth adapters for the host computer.

To get it to work, plug in the second adapter **after** the first 7 connections are made. This should allow you seven more connections.
