#!/bin/bash

set -u
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
INTERFACE_DIR=$SCRIPT_DIR/bulletInterface

mkdir -p $INTERFACE_DIR/build
cd $INTERFACE_DIR/build

# I use ninja, but just get rid of -GNinja for make instead
if [[ $1 ]]; then
		cmake -GNinja $INTERFACE_DIR -DSCENEGRAPH_INTERFACE=ON
else
		cmake -GNinja $INTERFACE_DIR -DSCENEGRAPH_INTERFACE=OFF
fi

ninja -j4

