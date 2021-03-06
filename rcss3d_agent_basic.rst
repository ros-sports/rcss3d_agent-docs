Rcss3d Agent Basic
##################

Rcss3d Agent Basic is a ROS2 Component that can run as a standalone application. The term
**Basic** comes from the node closely mimicking the interface provided by SimSpark.
Custom ROS2 interfaces are used with no attempt at converting the interfaces to those
you would probably use to communicate with a real robot.

.. image:: images/rcss3d_agent_basic_flowchart.png
  :align: center


.. seealso::

  If you wish to publish and subscribe to topics with different interfaces, you should
  use the rcss3d_agent library and implement the pub/sub yourself, see :doc:`rcss3d_agent`
  for more information.

Starting the Simulator
**********************

In a terminal, start the simulator by running:

.. code-block:: console

    rcsoccersim3d

.. tip::

    Simulator tends to crash sometimes when connecting / disconnecting agents, which leaves unwanted
    server processes lingering. To kill this process, run ``pkill -9 rcssserver3d`` before restarting
    the simulation server.

Launching a Player
******************

To run rcss3d_agent_basic as a standalone application, in a new terminal, run:

.. code-block:: console

  ros2 run rcss3d_agent_basic rcss3d_agent_basic

Moving around the simulator camera with the WASD keys and the mouse, 
you should see your robot at the corner of the field, as below:

.. image:: images/agent_in_simulator.png
  :align: center

Beaming the Robot
*****************

The `Beam Effector`_ allows a player to position itself on the field before the start of each half.
In this example, we will move the robot to four metres behind the centre circle, facing it. The
coordinate of the robot after beamed will be (-4.0, 0.0, 0.0).

In a new terminal, run:

.. code-block:: console

  ros2 topic pub --once effectors/beam rcss3d_agent_msgs/msg/Beam "
  x: -4.0
  y: 0.0
  rot: 0.0
  "

In the simulator, you should see the robot has moved to the requested pose as below:

.. image:: images/beamed_robot.png

Moving a Hinge Joint
********************

To send a hinge joint command to the simulated robot, you must publish
a ``rcss3d_agent_msgs/msg/HingeJointVel`` msg on the ``effectors/hinge_joint`` topic.

In this example, we will set the velocity of the head yaw to be 1.0. In a new terminal, run:

.. code-block:: console

  ros2 topic pub --once effectors/hinge_joint rcss3d_agent_msgs/msg/HingeJointVel "
  name: 'he1'
  ax: 1.0
  "

In the simulation, you should see the robot turn its head left until it hits the joint limit as
following:

.. image:: images/robot_turning_head.gif

.. seealso::

  For a list of the name of hinge joint effectors, refer to RoboCup 3D Simulation League's
  `Nao model`_.

Moving a Universal Joint
************************

To send a universal joint command to the simulated robot, you must publish
a ``rcss3d_agent_msgs/msg/UniversalJointVel`` msg on the ``effectors/universal_joint`` topic.

The default Nao robot model doesn't have any universal joints. The example below shows how to move
the universal shoulder joint of SimSpark's `SoccerBot`_:

.. code-block:: console

  ros2 topic pub --once effectors/hinge_joint rcss3d_agent_msgs/msg/HingeJointVel "
  name: 'lae1_2'
  ax1: 1.0
  ax2: -1.0
  "

Communicating with Other Agents
*******************************

The `Say Effector`_ permits communication among agents by broadcasting messages. Be sure to read
about this effector's dual, the `Hear Perceptor`_, as it details restrictions upon what message 
content may be sent, and under what circumstances other agents will actually hear your messages.

In this example, we will send a msg containing the string "helloworld":

.. code-block:: console

  ros2 topic pub --once effectors/say rcss3d_agent_msgs/msg/Say "message: 'helloworld'"

Synchronize Effector
********************

The `Synchronize Effector`_ must be used if the simulator is running with `Agent Sync Mode`_.
In Agent Sync Mode, agents must publish this message at the end of each simulation cycle.
Note that the server ignores this command if it is received in Real-Time Mode, so it is safe to
configure your agent to always publish this message.

In the example below, we will publish a message on the :code:`effectors/synchronize` topic
at 100Hz:

.. code-block:: console

  ros2 topic pub --rate 100 effectors/synchronize rcss3d_agent_msgs/msg/Synchronize

We can confirm that the simulator is responding to the synchronize message by printing the publish
rate of the :code:`/percept` topic with:

.. code-block:: console

  ros2 topic hz percept

The average rate should be close to the rate at we are sending our synchronize effector (ie. 100Hz).

Topics
******

List of topics used by the node.

Published Topics
================

* **percept** (*rcss3d_agent_msgs/msg/Percept*)

Subscribed Topics
=================

* **effectors/beam** (*rcss3d_agent_msgs/msg/Beam*)
* **effectors/hinge_joint** (*rcss3d_agent_msgs/msg/HingeJointVel*)
* **effectors/say** (*rcss3d_agent_msgs/msg/Say*)
* **effectors/universal_joint** (*rcss3d_agent_msgs/msg/UniversalJointVel*)
* **effectors/synchronize** (*rcss3d_agent_msgs/msg/Synchronize*)

Parameters
**********

List of parameters for the node.

* **model** (*string*, default="rsg/agent/nao/nao.rsg")

  The RSG model of the robot, model path must be relative to and inside
  :code:`/usr/local/share/rcssserver3d/`

* **rcss3d/host** (*string*, default="127.0.0.1")

  Host IP Address that simulation server is running on
    
* **rcss3d/port** (*int*, default=3100)

  Port number that simulation server is communicating on
    
* **team** (*string*, default="Anonymous")

  Team name of robot, to be sent to simulation server
    
* **unum** (*int*, default=0)

  Player number of robot, to be sent to simulation server

.. _Beam Effector: https://gitlab.com/robocup-sim/SimSpark/-/wikis/Effectors#beam-effector
.. _Nao model: https://gitlab.com/robocup-sim/SimSpark/-/wikis/Models#equipment
.. _Soccerbot: https://gitlab.com/robocup-sim/SimSpark/-/wikis/Models#soccerbot
.. _Say Effector: https://gitlab.com/robocup-sim/SimSpark/-/wikis/Effectors#say-effector
.. _Hear Perceptor: https://gitlab.com/robocup-sim/SimSpark/-/wikis/Perceptors#hear-perceptor
.. _Synchronize Effector: https://gitlab.com/robocup-sim/SimSpark/-/wikis/Effectors#synchronize-effector
.. _Agent Sync Mode: https://gitlab.com/robocup-sim/SimSpark/-/wikis/Agent-Sync-Mode