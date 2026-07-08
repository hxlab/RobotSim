#!/bin/bash

sudo chown -R $USER:$USER deps/olvx_descriptions_module
rm -rf deps/olvx_descriptions_module

sudo chown -R $USER:$USER deps/franka_description
rm -rf deps/franka_description

sudo chown -R $USER:$USER deps/libfranka
rm -rf deps/libfranka