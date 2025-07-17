#!/bin/bash

# Add environment variable exports to ~/.bashrc if they don't already exist
if ! grep -q "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc; then
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
    echo "Added RMW_IMPLEMENTATION export to ~/.bashrc"
fi

if ! grep -q "export CYCLONEDDS_URI=" ~/.bashrc; then
    echo "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>32</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'" >> ~/.bashrc
    echo "Added CYCLONEDDS_URI export to ~/.bashrc"
fi

# Source the bashrc file to apply changes to current session
source ~/.bashrc
echo "Sourced ~/.bashrc - environment variables are now active"
