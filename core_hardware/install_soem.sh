#!/bin/bash

# Get the directory of this script
CORE_HARDWARE_DIR=$(cd $(dirname $0) ; pwd)/
SOEM_DIR=$CORE_HARDWARE_DIR/vendor/soem

if [ -d $SOEM_DIR ]; then
    echo "SOEM is already installed."
    exit 0
fi

mkdir -p $SOEM_DIR
curl -L -o $SOEM_DIR/soem-2.0.0-Linux-x86_64.zip https://github.com/OpenEtherCATsociety/SOEM/releases/download/v2.0.0/soem-2.0.0-Linux-x86_64.zip
unzip $SOEM_DIR/soem-2.0.0-Linux-x86_64.zip -d $SOEM_DIR
mv $SOEM_DIR/soem-2.0.0-Linux-x86_64/* $SOEM_DIR/
rm -r $SOEM_DIR/soem-2.0.0-Linux-x86_64.zip
rm -rf $SOEM_DIR/soem-2.0.0-Linux-x86_64