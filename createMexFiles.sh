#!/bin/bash

set -u
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
INTERFACE_DIR=$SCRIPT_DIR/bulletComponents

mkdir -p $INTERFACE_DIR/build
cd $INTERFACE_DIR/build

cmake $INTERFACE_DIR
make -j4

