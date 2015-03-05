from __future__ import print_function
from collections import defaultdict
from Cython.Distutils import build_ext
from distutils.core import setup
from distutils.extension import Extension
import subprocess
import numpy
import sys
import platform
import os

# JB: faster compilation? remove for production
os.environ['CFLAGS'] = '-O0'
boudaries_extra_objects = ['pcl/_pcl.so']

if platform.system() == "Darwin":
    # Use default OS X ARCHFLAGS, otherwise Python adds unknown architectures.
    os.environ['ARCHFLAGS'] = '' 
    # Do not cross-compile extra objects, the OS X linker does not understand shared objects (SO)
    boundaries_extra_objects = []

# Try to find PCL. XXX we should only do this when trying to build or install.
PCL_SUPPORTED = ["-1.7", ""]    # in order of preference

for pcl_version in PCL_SUPPORTED:
    if subprocess.call(['pkg-config', 'pcl_common%s' % pcl_version]) == 0:
        break
else:
    print("%s: error: cannot find PCL, tried" % sys.argv[0], file=sys.stderr)
    for version in PCL_SUPPORTED:
        print('    pkg-config pcl_common%s' % version, file=sys.stderr)
    sys.exit(1)

# Find build/link options for PCL using pkg-config.
pcl_libs = ["common", "features", "filters", "io", "kdtree", "octree",
            "registration", "sample_consensus", "search", "segmentation",
            "surface"]
pcl_libs = ["pcl_%s%s" % (lib, pcl_version) for lib in pcl_libs]

ext_args = defaultdict(list)
ext_args['include_dirs'].append(numpy.get_include())

def pkgconfig(flag):
    # Equivalent in Python 2.7 (but not 2.6):
    #subprocess.check_output(['pkg-config', flag] + pcl_libs).split()
    p = subprocess.Popen(['pkg-config', flag] + pcl_libs,
                         stdout=subprocess.PIPE)
    stdout, _ = p.communicate()
    # Assume no evil spaces in filenames; unsure how pkg-config would
    # handle those, anyway.
    # decode() is required in Python 3. TODO how do know the encoding?
    return stdout.decode().split()


for flag in pkgconfig('--cflags-only-I'):
    ext_args['include_dirs'].append(flag[2:])
for flag in pkgconfig('--cflags-only-other'):
    if flag.startswith('-D'):
        macro, value = flag[2:].split('=', 1)
        ext_args['define_macros'].append((macro, value))
    else:
        ext_args['extra_compile_args'].append(flag)
for flag in pkgconfig('--libs-only-l'):
    if flag == "-lflann_cpp-gd":
        print("skipping -lflann_cpp-gd (see https://github.com/strawlab/python-pcl/issues/29")
        continue
    ext_args['libraries'].append(flag[2:])
for flag in pkgconfig('--libs-only-L'):
    ext_args['library_dirs'].append(flag[2:])
for flag in pkgconfig('--libs-only-other'):
    ext_args['extra_link_args'].append(flag)

# Hidden dependency of boundary detection.
ext_args['libraries'].append(u'boost_system')

setup(name='python-pcl',
      description='pcl wrapper',
      url='http://github.com/strawlab/python-pcl',
      version='0.2',
      author='John Stowers',
      author_email='john.stowers@gmail.com',
      license='BSD',
      packages=["pcl"],
      ext_modules=[Extension("pcl._pcl", ["pcl/_pcl.pyx", "pcl/minipcl.cpp"],
                             language="c++", **ext_args),
                   Extension("pcl.registration", ["pcl/registration.pyx", "pcl/registration_helper.cpp"],
                             language="c++", **ext_args),
                   Extension("pcl.boundaries", ["pcl/boundaries.pyx"], extra_objects=boundaries_extra_objects,
                             language="c++", **ext_args),
                  ],
      cmdclass={'build_ext': build_ext}
      )
