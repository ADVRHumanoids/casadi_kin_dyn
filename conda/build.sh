# specify install location
# 1. pybind, otherwise it installs using python 3.9
# 2. install in conda

echo -e "installing ..."

CMAKE_FLAGS="-DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DCMAKE_PREFIX_PATH=$CONDA_PREFIX"

# Build in subdirectory
mkdir build
cd build
cmake .. $CMAKE_FLAGS
make -j4
make install

echo -e "done."
