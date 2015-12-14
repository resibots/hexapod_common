# hexapod_planner

#### Here we keep the (path) planners that produce series of actions or paths for our hexapods to follow.

## Available planner

### HexapodPlannerSimple

A simple A* action planner.

**UNDER CONSTRUCTION**

## How to compile

### Compile and install

- cd to `hexapod_planner` folder
- Configure with `./waf configure --prefix=path_to_install`
- Compile with `./waf build`
- Install with `./waf install`

## How to use it in other projects

### Using the WAF build system

Add hexapod_planner as an external library using the following script:

```python
#! /usr/bin/env python
# encoding: utf-8
# Konstantinos Chatzilygeroudis - 2015

"""
Quick n dirty hexapod_planner detection
"""

import os
from waflib.Configure import conf


def options(opt):
	opt.add_option('--planner', type='string', help='path to planner', dest='planner')

@conf
def check_hexapod_controller(conf):
	includes_check = ['/usr/local/include/hexapod_planner', '/usr/include/hexapod_planner']

	# You can customize where you want to check
	# e.g. here we search also in a folder defined by an environmental variable
	if 'RESIBOTS_DIR' in os.environ:
		includes_check = [os.environ['RESIBOTS_DIR'] + '/include/hexapod_planner'] + includes_check

	if conf.options.planner:
		includes_check = [conf.options.planner + '/include/hexapod_planner']

	try:
		conf.start_msg('Checking for hexapod_planner includes')
		res = conf.find_file('hexapod_planner_simple.hpp', includes_check)
		res = res and conf.find_file('state_simple.hpp', includes_check)
		res = res and conf.find_file('simple_env.hpp', includes_check)
		conf.end_msg('ok')
		conf.env.INCLUDES_HEXAPOD_PLANNER = includes_check
	except:
		conf.end_msg('Not found', 'RED')
		return
	return 1
```

Then in your C++ code you would have something like the following:

```cpp
// previous includes
#include <hexapod_controller_planner.hpp>

// rest of code

HexapodPlannerSimple<StateSimple, RobotSimple, EnvironmentSimple<ObstacleSimple>> planner(actions, goal, env);

// rest of code
```


## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
