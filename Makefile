ifeq "$(OS)" ""
	OS = $(shell uname -s)
endif

ifeq "$(OS)" "Darwin"
	export ARCHFLAGS= -arch x86_64
endif

PCL_BIN = pcl/_pcl.so
REGISTRATION_BIN = pcl/registration.so

all: $(PCL_BIN) $(REGISTRATION_BIN)

$(PCL_BIN): pcl/_pcl.pxd pcl/_pcl.pyx setup.py pcl/pcl_defs.pxd \
             pcl/minipcl.cpp pcl/indexing.hpp
	python setup.py build_ext --inplace

$(REGISTRATION_BIN): setup.py pcl/_pcl.pxd pcl/pcl_defs.pxd \
                      pcl/registration.pyx
	python setup.py build_ext --inplace

test: $(PCL_BIN) tests/test.py
	nosetests -s

clean:
	rm -rf build
	rm -f pcl/_pcl.cpp $(PCL_BIN) $(REGISTRATION_BIN)

doc: pcl.so conf.py readme.rst
	sphinx-build -b singlehtml -d build/doctrees . build/html

showdoc: doc
	gvfs-open build/html/readme.html

