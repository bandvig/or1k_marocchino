#!/bin/bash

export HOME=/tmp

# Setup fusesoc and add the cores required by or1k-tests
fusesoc init -y
fusesoc library add mor1kx-generic https://github.com/stffrdhrn/mor1kx-generic.git
fusesoc library add intgen https://github.com/stffrdhrn/intgen.git
fusesoc library add or1k_marocchino /src

cd $HOME/src/tools

cd /src

echo "Running Job $JOB $SIM "
echo "Expected failures: $EXPECTED_FAILURES"

./.travis/run-${JOB}.sh