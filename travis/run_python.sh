#!/bin/bash
set -ev

cd $FOREST_DIR

# required due to shell processes (that spawn independently every .sh file)!
source setup.bash
echo $CMAKE_PREFIX_PATH

# required packages for building casadi_kin_dyn
FOREST_ARGS="--default-build-type Release --clone-protocol https --verbose -j2"

forest grow casadi_kin_dyn $FOREST_ARGS
cd build/casadi_kin_dyn
make generate_python_package
cd ../../src/casadi_kin_dyn

# upload if push is tag
if [ -z $TRAVIS_TAG ]; then
    echo "Not a tag build, will not upload to pypi";
else
    twine upload --verbose -u __token__ -p $PYPI_TOKEN dist/*.whl;
fi
