# GenNav 

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://travis-ci.org/ERC-BPGC/gennav.svg?branch=master)](https://travis-ci.org/ERC-BPGC/gennav)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/ERC-BPGC/gennav.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/ERC-BPGC/gennav/context:python)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/ERC-BPGC/gennav.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/ERC-BPGC/gennav/alerts/)
[![Documentation Status](https://readthedocs.org/projects/gennav/badge/?version=latest)](https://gennav.readthedocs.io/en/latest/?badge=latest)

**Status: Under Development**

A python package for robot navigation algorithms.

## Installation

The package is currently under development so we suggest installing from source.

### From Source (Recommended)
```bash
git clone https://github.com/ERC-BPGC/gennav.git
cd gennav
python -m pip install .
```

### Using pip
```bash
pip install gennav
```

## Usage

To plan a path using the Rapidly-exploring random tree algorithm in a polygon based environment representation. 

```python
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
```

Note that the environment have been left blank empty here, they should be updated as per use case.

For more details you can refer to our [documentation](https://gennav.readthedocs.io/en/latest/index.html).

## Contributions

Contributions are always welcome. We reccomend you check out [contribution guidelines](./CONTRIBUTION.md) and view the [docs](https://gennav.readthedocs.io/en/latest/index.html) beforehand.
