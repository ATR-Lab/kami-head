#!/bin/bash

# Script to fix Qt version conflicts before running the actual application

# Unset potentially conflicting Qt environment variables
unset QT_PLUGIN_PATH
unset QML2_IMPORT_PATH

# Force the system Qt version (5.15.13)
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# Run the original application with all arguments passed to this script
# Replace the command below with your actual application command
"$@" 