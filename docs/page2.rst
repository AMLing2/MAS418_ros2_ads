Ros2 with ADS - Green crane
==============

The project involves simulating a crane operation controlled by a Programmable Logic Controller (PLC). The PLC communicates with Robot Operating System 2 (ROS2) using the Automation Device Specification (ADS). A visualization of the crane was done using RViz, and with a publishing node which listens to a ROS2 message interface.

The project was done with a CX2033 PLC, and a computer running ROS2 jazzy in a virtual machine with Ubuntu 24.04 LTS.


Usage
---------
First source ROS2, for the publisher to build, the 'src/crane_interface' package must be build and sourced first: 

.. code-block:: bash

    colcon build

This can then be sourced to add the interface to ROS2:
.. code-block:: bash

    source install/local_setup.bash

To run the ROS2 nodes, first for the publisher 'src/crane_publisher':

.. code-block:: bash

    ros2 run crane_publisher crane_publisher_cpp

Visualizer and ADS listener which is in the 'src/green_crane_urdf' folder, which is done through a launch file:

.. code-block:: bash

    ros2 launch green_crane_urdf display.launch.py


PLC Setup
---------

The plc was set up including a simulation generated from a Simulink file, altough not fully implemented, manual and automatic modes were created which implemented a function block interface, using function block pointers to switch between them. No Global Variable Lists (GVL) were used, instead variables were placed in the main program and in function blocks.

In the PLC, the network ID of the ubuntu virtual machine was specified, which was set to '192.16.0.16'. For this to work, a second network interface had to be created in VirtualBox, and in the virtual machine the ethernet interface had to be set to the same specified IP. The firewalls did not have to be re-configured for this to work.

Communication with ROS2 Using ADS
---------------------------------

ADS is used for efficient communication between the PLC and external systems, such as ROS2. It facilitates secure data exchange and command transmission.

- **Setting Up ADS in ROS2**: 

Initially, the Beckhoff ADS library was incorporated into the project as a Git submodule, positioned within the `ADS` directory. Within the `CMakeLists.txt` file for the nodes, the path to the ADS library was specified. This configuration ensured that ADS functionalities were appropriately compiled and linked with the project's core components. The ADS I/O was configured to run within ROS2 nodes, either reading data from the 'MAIN' function block in the PLC program, or writing to them.

To establish ADS communication within the ROS2 framework, an ADS client was instantiated within the main program, which were then passed with a reference to the ROS2 nodes. This client was configured the network ID corresponding to the PLC. For the publisher, a new message interface was created which included the relevant variables to set. This was done so other components in ROS2 can communicate with the PLC without needing to use ADS directly. The visualization was set up with RViz, which used 'Joint State Publisher' to send data to RViz.

It was however found out that two ADS clients could not be run at the same time, there fore the publisher/subscriber had to be stopped and restarted when using them separately. Optimally the I/O of the ADS should be put into a single node which only recieves and sends the data from/to the ADS network, therefore placing the visualization in another node.

A launch file was made for the visualization, which open RViz with the correct URDF file, as well as starting the ADS reciever and joint state publisher node.


Video showcase
-------------
In the video, the message to enable the simulator is first sent through ROS2 terminal commands, this can be seen updating the PLC variable. Crane controls are then sent.

.. raw:: html

   <video width="640" height="480" controls>
     <source src="_static/ros2.mp4" type="video/mp4">
     Your browser does not support the video tag.
   </video>
