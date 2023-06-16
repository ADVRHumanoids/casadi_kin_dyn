#!/bin/bash -i
# -i stands for interactive mode: required to make effective source ~/.bashrc
set -v

curl -L -O https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-$(uname)-$(uname -m).sh
printf '\nyes\n\nyes\n' | bash Mambaforge-$(uname)-$(uname -m).sh


# re-activate shell
export PATH=$PWD/mambaforge/bin:$PATH
source ~/.bashrc

echo -e "creating conda environment.."

# create environment for conda
yes Y | mamba env create -f conda/env_build.yml

echo -e "done."


