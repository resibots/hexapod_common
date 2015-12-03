# hexapod_controller

#### Here we keep the gait controllers (generators) that make our hexapods alive.

## Available controllers

### HexapodControllerSimple

A simple sinusoidal controller.

**UNDER CONSTRUCTION**

## How to compile

### Compile and install

- cd to `hexapod_common` folder
- Configure with `./waf configure --prefix=path_to_install`
- Compile with `./waf build`
- Install with `./waf install`

## How to use it in other projects

### Using the WAF build system

Add hexapod_controller as an external library using the following script:

```python
#! /usr/bin/env python
# encoding: utf-8
# Konstantinos Chatzilygeroudis - 2015

"""
Quick n dirty hexapod_controller detection
"""

import os
from waflib.Configure import conf


def options(opt):
	opt.add_option('--controller', type='string', help='path to hexapod_controller', dest='controller')

@conf
def check_hexapod_controller(conf):
	includes_check = ['/usr/local/include/hexapod_controller', '/usr/include/hexapod_controller']
	libs_check = ['/usr/local/lib', '/usr/lib']

	# You can customize where you want to check
	# e.g. here we search also in a folder defined by an environmental variable
	if 'RESIBOTS_DIR' in os.environ:
		includes_check = [os.environ['RESIBOTS_DIR'] + '/include/hexapod_controller'] + includes_check
		libs_check = [os.environ['RESIBOTS_DIR'] + '/lib'] + libs_check

	if conf.options.controller:
		includes_check = [conf.options.controller + '/include/hexapod_controller']
		libs_check = [conf.options.controller + '/lib']

	try:
		conf.start_msg('Checking for hexapod_controller includes')
		res = conf.find_file('hexapod_controller_simple.hpp', includes_check)
		conf.end_msg('ok')
		conf.start_msg('Checking for hexapod_controller libs')
		res = res and conf.find_file('libhexapod_controller_simple.so', libs_check)
		conf.end_msg('ok')
		conf.env.INCLUDES_HEXAPOD_CONTROLLER = includes_check
		conf.env.LIBPATH_HEXAPOD_CONTROLLER = libs_check
		conf.env.LIB_HEXAPOD_CONTROLLER = ['hexapod_controller_simple']
	except:
		conf.end_msg('Not found', 'RED')
		return
	return 1
```

Then in your C++ code you would have something like the following:

```cpp
// previous includes
#include <hexapod_controller_simple.hpp>

// rest of code

HexapodControllerSimple controller(controller_parameters, broken_legs);

// rest of code
```


## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html