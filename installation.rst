Installation
############

.. warning::

  The binary installation is not available yet. This warning will be removed when the
  upcoming the binary installation becomes available.

Binary Installation
*******************

.. code-block:: console
  
  sudo apt update && sudo apt install ros-rolling-rcss3d-agent-basic


Source Installation
*******************

ROS2 Foxy onwards is supported.

.. note::

   Instructions here assume that you have and are in a ROS2 workspace's
   root directory.

In your ROS2 workspace parent directory, run:

.. code-block:: console

   git clone --recursive https://github.com/ros-sports/rcss3d_agent.git src/rcss3d_agent
   colcon build
