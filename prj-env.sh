#!/bin/bash

## Export the project directory environment variable
script_path=`realpath $0`
script_path=${script_path%/*}
export PRJ_DIR=${script_path}

## Source the zephyr environment configuration
source $PRJ_DIR/../3rd-party/zephyr/zephyr-env.sh
