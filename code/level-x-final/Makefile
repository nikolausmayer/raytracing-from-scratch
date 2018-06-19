##
## Author: Nikolaus Mayer, 2018 (mayern@cs.uni-freiburg.de)
##

CXX ?= g++

SRC = main.cpp

.PHONY: clean default debug

default: raytracer

raytracer: Makefile $(SRC)
	$(info Building raytracer...)
	$(CXX) -O3 -std=c++11 -o raytracer main.cpp 

debug: Makefile $(SRC)
	$(info Building raytracer... (debug mode))
	$(CXX) -O0 -g -std=c++11 -o raytracer main.cpp 

clean:
	-rm raytracer

