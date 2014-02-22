from distutils.core import setup, Extension

data_module = Extension('cindata',
                        sources = ['cindata_module.c'])

setup (name = 'pycin', 
       version = '1.0',
       description = 'Python bindings for libcin',
       ext_modules = [data_module])
