#!/bin/bash

sudo chown -R $USER:$USER franka_deps/olvx_descriptions_module
rm -rf franka_deps/olvx_descriptions_module

sudo chown -R $USER:$USER franka_deps/franka_description
rm -rf franka_deps/franka_description

sudo chown -R $USER:$USER franka_deps/libfranka
rm -rf franka_deps/libfranka