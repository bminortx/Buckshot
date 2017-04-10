#!/bin/bash

set -u
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

DEPS_DIR=$SCRIPT_DIR/dependencies
mkdir -p $DEPS_DIR
cd $DEPS_DIR

# Get Bullet3
# git clone https://github.com/bulletphysics/bullet3.git
# cd bullet3
# mkdir build
# cd build
# cmake .. -DUSE_DOUBLE_PRECISION=ON
# make -j4
# sudo make install

# Get FreeGLUT
cd $DEPS_DIR
# If you're on a mac...
brew install Caskroom/cask/xquartz
brew install homebrew/x11/freeglut
brew install cmake

# else...
# curl -o freeglut-3.0.0.tar.gz 'http://skylineservers.dl.sourceforge.net/project/freeglut/freeglut/3.0.0/freeglut-3.0.0.tar.gz'
# tar xvcf freeglut-3.0.0.tar.gz
# cd freeglut-3.0.0
# mkdir build
# cd build
# cmake ..





