#! /usr/bin/env python
# encoding: utf-8
# Konstantinos Chatzilygeroudis - 2015

"""
Quick n dirty SFML detection
"""

import os
from waflib.Configure import conf


def options(opt):
	opt.add_option('--sfml', type='string', help='path to SFML', dest='sfml')

@conf
def check_sfml(conf):
	includes_check = ['/usr/local/include', '/usr/include']
	libs_check = ['/usr/local/lib', '/usr/lib', '/usr/lib/x86_64-linux-gnu']
	if 'RESIBOTS_DIR' in os.environ:
		includes_check = [os.environ['RESIBOTS_DIR'] + '/include'] + includes_check
		libs_check = [os.environ['RESIBOTS_DIR'] + '/lib'] + libs_check

	if conf.options.sfml:
		includes_check = [conf.options.sfml + '/include']
		libs_check = [conf.options.sfml + '/lib']

	sfml_includes = ['Audio.hpp', 'Config.hpp', 'Graphics.hpp', 'Network.hpp', 'System.hpp', 'Window.hpp']
	sfml_libs = ['sfml-audio', 'sfml-graphics', 'sfml-network', 'sfml-system', 'sfml-window']

	try:
		conf.start_msg('Checking for SFML includes')
		res = True
		for f in sfml_includes:
			res = res and conf.find_file('SFML/'+f, includes_check)
		conf.end_msg('ok')
		conf.start_msg('Checking for SFML libs')
		for f in sfml_libs:
			res = res and conf.find_file('lib'+f+'.so', libs_check)
		conf.end_msg('ok')
		conf.env.INCLUDES_SFML = includes_check
		conf.env.LIBPATH_SFML = libs_check
		conf.env.LIB_SFML = sfml_libs
	except:
		conf.end_msg('Not found', 'RED')
		return
	return 1