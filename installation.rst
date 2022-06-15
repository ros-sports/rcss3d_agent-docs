Installation
############

To install the packages, do one of the following:

* a `Binary Installation`_
* a `Source Installation`_

Binary Installation
*******************

Binary installation is available for ROS2 Galactic onwards.

Source your ROS installation, then run:

.. code-block:: console

  sudo apt update
  sudo apt install ros-${ROS_DISTRO}-rcss3d-agent-basic

If this method does not work for your platform, perform the `Source Installation`_ instead.

Source Installation
*******************

Source installation works for ROS2 Foxy onwards.

Source your ROS installation, then run the following in your ROS workspace:

.. code-block:: console

   git clone https://github.com/ros-sports/rcss3d_agent.git src/rcss3d_agent --branch ${ROS_DISTRO}
   rosdep install --from-paths src -i
   colcon build

Confirming Installation
***********************

To confirm that the packages have installed correctly, source either your:

* ros workspace if you did the binary installation (ie. :code:`source /opt/ros/${ROS_DISTRO}/setup.bash`)
* current workspace if you did the source installation (ie. :code:`source install/local_setup.bash`)

List installed ROS2 packages with:

.. code-block:: console

  ros2 pkg list | grep rcss3d_agent

Your installation worked correctly if you see rcss3d_agent packages listed below:

.. code-block:: console

  rcss3d_agent
  rcss3d_agent_basic
  rcss3d_agent_msgs

