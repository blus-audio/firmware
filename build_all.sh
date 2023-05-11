#!/bin/bash
set -e

# Build all apps in the chosen directory with make.
APPSDIR=./apps

pushd $APPSDIR
APPS=$(ls -d */)
popd

for app in $APPS;
do
    make APP=$(basename $app)
done
