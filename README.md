# GenNav 

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://travis-ci.org/ERC-BPGC/gennav.svg?branch=master)](https://travis-ci.org/ERC-BPGC/gennav)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/ERC-BPGC/gennav.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/ERC-BPGC/gennav/context:python)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/ERC-BPGC/gennav.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/ERC-BPGC/gennav/alerts/)
[![Documentation Status](https://readthedocs.org/projects/gennav/badge/?version=latest)](https://gennav.readthedocs.io/en/latest/?badge=latest)

**Status: Under Development**

A Python Package for Robot navigation algorithms.

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
```

Note that the environment have been left blank empty here, they should be updated as per use case.

For more details you can refer to our [documentation](https://gennav.readthedocs.io/en/latest/index.html).

## Contributions

Contributions are always welcome. We reccomend you check out [contribution guidelines](./CONTRIBUTION.md) and view the [docs](https://gennav.readthedocs.io/en/latest/index.html) beforehand.
