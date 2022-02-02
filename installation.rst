Installation
############

To install the packages, do one of the following:

* a `Binary Installation`_
* a `Source Installation`_

Binary Installation
*******************

.. warning::

  The binary installation is not available yet. This warning will be removed when the
  upcoming binary installation becomes available.

Rcss3d Agent is released to the ROS ecosystem. You can install the debian packages using apt
as following:

.. code-block:: console
  
  sudo apt update && sudo apt install ros-rolling-rcss3d-agent-basic

If this method does not work for your platform, perform the `Source Installation`_ instead.

Source Installation
*******************

Source installation should work for almost any platform that has ROS2 installed on it.
ROS2 Foxy onwards is supported. There is no alternative for ROS1 distros or older ROS2 distros.

From within a ROS2 workspace, run the following code that clones Rcss3d Agent into your
src directory, and builds your workspace:

.. code-block:: console

   git clone --recursive https://github.com/ros-sports/rcss3d_agent.git src/rcss3d_agent
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

