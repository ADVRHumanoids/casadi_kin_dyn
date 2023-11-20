#!/bin/bash -i
pip3 install build twine hhcm-forest==1.1.14
SRC_FOLDER=$PWD
cd .. && mkdir forest_ws && cd forest_ws && forest init  # create forest ws for building
# source setup.bash
ln -s $SRC_FOLDER src/$(basename $SRC_FOLDER)  # symlink original source folder

ls ../casadi_kin_dyn/travis/

cp -R ../casadi_kin_dyn/travis/recipes .

# FOREST_TAG='master'
# forest add-recipes git@github.com:advrhumanoids/multidof_recipes.git --tag $FOREST_TAG --clone-protocol https  # get recipes from repo
