# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

# def options(opt):
#     pass

# def configure(conf):
    # conf.check_nonfatal(header_name='stdint.h', define_name='HAVE_STDINT_H')

def build(bld):
    module = bld.create_ns3_module('rosns3', ['aodv','mobility','core','network','visualizer','olsr','applications'])
    module.includes = '/home/malintha/Desktop/fb/flatbuffers/include'
    module.source = [
        'model/rosns3.cc',
        'helper/rosns3-helper.cc',
        ]

    module_test = bld.create_ns3_module_test_library('rosns3')
    module_test.source = [
        'test/rosns3-test-suite.cc',
        ]

    headers = bld(features='ns3header')
    headers.module = 'rosns3'
    headers.source = [
        'model/rosns3.h',
        'helper/rosns3-helper.h',
        ]

    if bld.env.ENABLE_EXAMPLES:
        bld.recurse('examples')
    
    bld.env.CXXFLAGS += ['-Wno-error','-Wno-sign-compare', '-Wno-unused-variable']
    
    # bld.ns3_python_bindings()

