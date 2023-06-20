#!/bin/bash -i
set -ev

# A simple script to initialize the conda environment
ENVIRONMENT_NAME=kindyn
PACKAGE_NAME=casadi_kin_dyn

if [ -z $TRAVIS_TAG ]; then
    SET_UPLOAD="--no-anaconda-upload"
    echo -e "Not a tag build, will not upload: $SET_UPLOAD";
else
    SET_UPLOAD="--token $CONDA_TOKEN"
    echo -e "uploading to conda: $SET_UPLOAD";
fi


# sourcing base conda path to activate environment
source $(mamba info --base)/etc/profile.d/conda.sh
echo -e "activating conda environment $ENVIRONMENT_NAME .."
mamba activate $ENVIRONMENT_NAME
echo -e "done."

echo -e "building package .."

cd conda/

# building the casadi_kin_dyn package
mamba build $SET_UPLOAD -c conda-forge .

echo -e "done."