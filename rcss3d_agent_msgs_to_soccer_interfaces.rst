Rcss3D Agent Msgs to Soccer Interfaces
######################################

This is a ROS2 package that provides a C++ library that allows the conversion of rcss3d agent messages to soccer interfaces messages.

Introduction
************

`Soccer Interfaces`_ is a collection of ROS2 interface packages for the soccer domain.
Converting simulator specific messages to more commonly used interfaces allows teams from other leagues to reuse the 3D simulator.

.. note::

  For 3D simulation teams looking to write code specifically for their league, this library isn't much use.

.. _Soccer Interfaces: https://soccer-interfaces.readthedocs.io/


Usage
*****

To use this package in your ament cmake project, you must declare it as a dependency in your package, as following:

.. code-block:: cpp

  // In your package.xml
  <depend>rcss3d_agent_msgs_to_soccer_interfaces</depend>

  // In your CMakeLists.txt
  find_package(rcss3d_agent_msgs_to_soccer_interfaces REQUIRED)
  ament_target_dependencies(your_target rcss3d_agent_msgs_to_soccer_interfaces)


To use the library, simply include the following header file:

.. code-block:: cpp

  #include "rcss3d_agent_msgs_to_soccer_interfaces/conversion.hpp"

API
***

The library provides the following functions for converting rcss3d agent messages to soccer interfaces messages.

Ball
====

.. code-block:: cpp

  soccer_vision_3d_msgs::msg::BallArray getBallArray(
    const std::optional<rcss3d_agent_msgs::msg::Ball> & ball);

Converts an optional ball message to a ball array message.

Header frame id is set to ``CameraTop_frame``.

Goalposts
=========

.. code-block:: cpp

  soccer_vision_3d_msgs::msg::GoalpostArray getGoalpostArray(
    const std::vector<rcss3d_agent_msgs::msg::Goalpost> & goalpost);

Converts a vector of goalpost messages to a goalpost array message.

Simulation reports the point at the top of the goalpost in spherical coordinates as observed from the robot camera.
Soccer Interfaces expects the center of the goalpost in cartesian coordinates as observed from the robot camera.

With no orientation of the goalpost available, it's impossible to correctly determine the center of the goalpost.
So we estimate the center of the goalpost by first converting spherical to cartesian coordinates, then subtracting half the height of the goalpost from the z coordinate.

The bounding box size is set to 0.1m x 0.1m x 0.8m such that it encapsulates the goalpost.

Header frame id is set to ``CameraTop_frame``.

Markings (FieldLines)
=====================

.. code-block:: cpp

  soccer_vision_3d_msgs::msg::MarkingArray getMarkingArray(
    const std::vector<rcss3d_agent_msgs::msg::FieldLine> & fieldLines);

Converts a vector of field line messages to a marking array message.
Since only field lines are detected in simulation, the ``segments`` field of the MarkingArray is populated, but ``ellipses`` and ``intersections`` are left empty.

Header frame id is set to ``CameraTop_frame``.

Robots
======

.. code-block:: cpp

  soccer_vision_3d_msgs::msg::RobotArray getRobotArray(
    const std::vector<rcss3d_agent_msgs::msg::Player> & players);

Converts a vector of player messages to a robot array message.
Only acknowledges a player as a robot if the head of the player was detected.
Players with only other body parts reported will be ignored, simply because it is difficult to determine the position of the robot when the head is not detected.

Similar to the goalpost, an approximate center of the robot is estimated by first converting spherical to cartesian coordinates, then subtracting half the height of the robot from the z coordinate.

The bounding box size is set to 0.3m x 0.3m x 0.6m such that it encapsulates the robot.

Header frame id is set to ``CameraTop_frame``.
