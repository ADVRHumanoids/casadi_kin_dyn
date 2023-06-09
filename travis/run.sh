#!/bin/bash
set -ev

# run python or conda distribution
if [ "$DISTRIBUTION" = "python" ]; then
    ./travis/run_python.sh;
fi

if [ "$DISTRIBUTION" = "conda" ]; then
    ./travis/run_conda.sh;
fi
