#!/usr/bin/env python

# Setup script for GDAL Python bindings.
# Inspired by psycopg2 setup.py file
# http://www.initd.org/tracker/psycopg/browser/psycopg2/trunk/setup.py
# Howard Butler hobu.inc@gmail.com

import sys
import os
import string

from glob import glob

# ---------------------------------------------------------------------------
# Switches
# ---------------------------------------------------------------------------

HAVE_NUMPY=False
HAVE_SETUPTOOLS = False
BUILD_FOR_CHEESESHOP = False

# ---------------------------------------------------------------------------
# Helper Functions
# ---------------------------------------------------------------------------

# Function to find numpy's include directory
def get_numpy_include():
    if HAVE_NUMPY:
        return numpy.get_include()
    else:
        return '.'


# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------

try:
    import numpy
    HAVE_NUMPY = True
    # check version
    numpy_major = numpy.__version__.split('.')[0]
    if int(numpy_major) < 1:
        print "numpy version must be > 1.0.0"
        HAVE_NUMPY = False
    else:
        print 'numpy include', get_numpy_include()
        if get_numpy_include() =='.':
            print "numpy headers were not found!  Array support will not be enabled"
            HAVE_NUMPY=False
except ImportError:
    pass



try:
    from setuptools import setup
    from setuptools import Extension
    HAVE_SETUPTOOLS = True
except ImportError:
    from distutils.core import setup, Extension


from distutils.command.build_ext import build_ext
from distutils.ccompiler import get_default_compiler
from distutils.sysconfig import get_python_inc


import popen2


def get_gdal_config(option, gdal_config='gdal-config'):

    command = gdal_config + " --%s" % option
    p = popen2.popen3(command)
    r = p[0].readline().strip()
    if not r:
        raise Warning(p[2].readline())
    return r  
    
class gdal_ext(build_ext):

    GDAL_CONFIG = 'gdal-config'
    user_options = build_ext.user_options[:]
    user_options.extend([
        ('gdal-config=', None,
        "The name of the gdal-config binary and/or a full path to it"),
    ])

    def initialize_options(self):
        build_ext.initialize_options(self)

        self.numpy_include_dir = get_numpy_include()
        self.gdaldir = None
        self.gdal_config = self.GDAL_CONFIG

    def get_compiler(self):
        return self.compiler or get_default_compiler()
    
    def get_gdal_config(self, option):
        return get_gdal_config(option, gdal_config =self.gdal_config)
    
    def finalize_win32(self):
        if self.get_compiler() == 'msvc':
            self.libraries.remove('gdal')
            self.libraries.append('gdal_i')
        build_ext.finalize_win32()
    
    def finalize_options(self):
        build_ext.finalize_options(self)
        
        self.include_dirs.append(self.numpy_include_dir)
        
        if self.get_compiler() == 'msvc':
            return True
        try:
            self.gdaldir = self.get_gdal_config('prefix')
            self.library_dirs.append(os.path.join(self.gdaldir,'lib'))
            self.include_dirs.append(os.path.join(self.gdaldir,'include'))
        except:
            print 'Could not run gdal-config!!!!'

gdal_version = '1.5.0.a.dev'


extra_link_args = []        
extra_compile_args = []

# might need to tweak for Python 2.4 on OSX to be these
#extra_compile_args = ['-g', '-arch', 'i386', '-isysroot','/']

gdal_module = Extension('osgeo._gdal',
                        sources=['extensions/gdal_wrap.cpp'],
                        extra_compile_args = extra_compile_args,
                        extra_link_args = extra_link_args)

gdalconst_module = Extension('osgeo._gdalconst',
                    sources=['extensions/gdalconst_wrap.c'],
                    extra_compile_args = extra_compile_args,
                    extra_link_args = extra_link_args)

osr_module = Extension('osgeo._osr',
                    sources=['extensions/osr_wrap.cpp'],
                    extra_compile_args = extra_compile_args,
                    extra_link_args = extra_link_args)

ogr_module = Extension('osgeo._ogr',
                    sources=['extensions/ogr_wrap.cpp'],
                    extra_compile_args = extra_compile_args,
                    extra_link_args = extra_link_args)


array_module = Extension('osgeo._gdal_array',
                    sources=['extensions/_gdal_array.cpp'],
                    extra_compile_args = extra_compile_args,
                    extra_link_args = extra_link_args)

ext_modules = [gdal_module,
              gdalconst_module,
              osr_module,
              ogr_module]

py_modules = ['gdal',
              'ogr',
              'osr',
              'gdalconst']
      
if HAVE_NUMPY:
    ext_modules.append(array_module)
    py_modules.append('gdalnumeric')

packages = ["osgeo",]

readme = file('README.txt','rb').read()

name = 'GDAL'
version = gdal_version
author = "Frank Warmerdam"
author_email = "warmerdam@pobox.com"
maintainer = "Howard Butler"
maintainer_email = "hobu.inc@gmail.com"
description = "GDAL: Geospatial Data Abstraction Library"
license = "MIT"
url="http://www.gdal.org"

classifiers = [
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
        'Programming Language :: Python',
        'Programming Language :: C',
        'Programming Language :: C++',
        'Topic :: Scientific/Engineering :: GIS',
        'Topic :: Scientific/Engineering :: Information Analysis',
        
]



if BUILD_FOR_CHEESESHOP:
    data_files = [("osgeo/data/gdal", glob(os.path.join("../../data", "*")))]
else:
    data_files = None
    
exclude_package_data = {'':['GNUmakefile']}

if HAVE_SETUPTOOLS:  
    setup( name = name,
           version = gdal_version,
           author = author,
           author_email = author_email,
           maintainer = maintainer,
           maintainer_email = maintainer_email,
           long_description = readme,
           description = description,
           license = license,
           classifiers = classifiers,
           py_modules = py_modules,
           packages = packages,
           url=url,
           data_files = data_files,
           zip_safe = False,
           exclude_package_data = exclude_package_data,
           install_requires =['numpy>=1.0.0'],
           cmdclass={'build_ext':gdal_ext},
           ext_modules = ext_modules )
else:
    setup( name = name,
           version = gdal_version,
           author = author,
           author_email = author_email,
           maintainer = maintainer,
           maintainer_email = maintainer_email,
           long_description = readme,
           description = description,
           license = license,
           classifiers = classifiers,
           py_modules = py_modules,
           packages = packages,
           data_files = data_files,
           url=url,
           cmdclass={'build_ext':gdal_ext},
           ext_modules = ext_modules )
