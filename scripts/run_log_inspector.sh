#!/usr/bin/env bash

source "$(dirname "$(realpath $0)")/lib/python_venv.sh" # Load python virtual enviroment must be ran before pushd
pushd "$(dirname "$(realpath $0)")" > /dev/null

pushd ../python/inertialsense/logInspector > /dev/null
PYTHONPATH=../../ python -m inertialsense.logInspector
popd > /dev/null

popd > /dev/null
