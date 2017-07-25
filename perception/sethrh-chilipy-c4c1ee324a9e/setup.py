import os
import subprocess
from distutils.core import setup, Extension
from Cython.Distutils import build_ext

# Numpy knows where it's include dir is
import numpy
numpy_include_dir = numpy.get_include()

# Get OpenCV libs & includes via pkg-config
opencv_libs = subprocess.check_output("pkg-config --libs opencv".split())
opencv_libs = [a[2:] for a in opencv_libs.split()] # remove leading "-l"
opencv_lib_dir = opencv_libs.pop(0)
opencv_includes = subprocess.check_output("pkg-config --cflags opencv".split())
opencv_includes = [a[2:] for a in opencv_includes.split()] # remove leading -I

# Chilitags
# I don't have this "installed" yet, so just put in the raw path
chilitags_include_dir = '/Users/seth/root/open-source/chilitags/include'
chilitags_lib_dir = '/Users/seth/root/open-source/chilitags/src'

include_dirs = [numpy_include_dir, chilitags_include_dir] + opencv_includes
libraries = ['chilitags'] + opencv_libs 
libraries_dirs = [chilitags_lib_dir, opencv_lib_dir]

ext_opts = {
    'include_dirs': include_dirs,
    'libraries': libraries,
    'library_dirs': libraries_dirs,
    'sources': ['chilipy.pyx'],
    'language': 'c++',
    'extra_compile_args': [
    # Ignore error about RW strings, struct[] = {"c string constant", ...}
    # '-Wno-c++11-compat-deprecated-writable-strings',
    # '-ferror-limit=1',
        '-DPYTHON_USE_NUMPY'
    ]
}

chilipy = Extension('chilipy', **ext_opts)

setup(name='chilipy', version='0.1', 
      description='Python bindings for Chilitags', 
      cmdclass={'build_ext': build_ext},
      ext_modules=[chilipy])
