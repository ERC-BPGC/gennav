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

   from gennav import planners, envs, utils
   from gennav.utils import RobotState
   from gennav.utils.geometry import Point
   from gennav.utils.samplers import uniform_random_sampler as sampler


   obstacles = []
   env = envs.PolyEnv()
   env.update(obstacles)

   start = RobotState(position=Point(1, 1))
   goal = RobotState(position=Point(10, 10))

   planner = RRT(sample_area=(-5, 15), sampler=sampler, expand_dis=0.1)
   path = planner.plan(start, goal, env)
