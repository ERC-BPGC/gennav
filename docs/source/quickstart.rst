Quickstart
==================================


Requirements
------------

GenNav is a purely python based library. We currently support both Python 2.7
and all later versions. 


Installation
------------

The package is currently under development so we suggest installing from source.

From Source (Recommended)
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   git clone https://github.com/ERC-BPGC/gennav.git
   cd gennav
   python -m pip install .


Using pip
~~~~~~~~~

.. code-block:: bash

   python -m pip install gennav


Usage
-----

To plan a path using the Rapidly-exploring random tree algorithm in a polygon
based environment representation. 

.. code-block:: python

   from gennav.planners import RRT
   from gennav.envs import PolygonEnv
   from gennav.utils import RobotState
   from gennav.utils.geometry import Point
   from gennav.utils.samplers import UniformRectSampler

   obstacles = []  # obstacles can be added here
   env = PolygonEnv()  # polygon environment to keep track of obstacles
   env.update(obstacles)  # updating environment with obstacles

   sampler = UniformRectSampler(-5, 15, -5, 15)  # for sampling random states
   planner = RRT(sampler=sampler, expand_dis=0.1)  # creating the planner

   start = RobotState(position=Point(1, 1))  # starting state
   goal = RobotState(position=Point(10, 10))  # goal state

   path = planner.plan(start, goal, env)  # planning path through obstacles
