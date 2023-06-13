#!/bin/bash -i
set -ev

# A simple script to initialize the conda environment
ENVIRONMENT_NAME=kindyn
PACKAGE_NAME=casadi_kin_dyn

if [ -z $TRAVIS_TAG ]; then
    echo -e "uploading to conda: false";
else
    echo -e "uploading to conda: true";
    mamba config --set anaconda_upload yes;
fi

# sourcing base conda path to activate environment
source $(mamba info --base)/etc/profile.d/conda.sh
echo -e "activating conda environment $ENVIRONMENT_NAME .."
mamba activate $ENVIRONMENT_NAME
echo -e "done."

echo -e "building package .."

cd conda/

# building the casadi_kin_dyn package
mamba build --token $CONDA_TOKEN -c conda-forge .

echo -e "done."

