#!/bin/bash

mkdir -p bulletWrapper/build
cd bulletWrapper/build
if [[ $1 ]]; then
		cmake .. -DSCENEGRAPH_INTERFACE=ON

else
		cmake .. -DSCENEGRAPH_INTERFACE=OFF
fi

make -j4

