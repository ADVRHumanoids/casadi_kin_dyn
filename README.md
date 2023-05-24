# casadi_kin_dyn
Package for generation of symbolic (SX) expressions of robot kinematics and dynamics. Based on URDF and Pinocchio.

## Dependencies
- `casadi` 3.4.5, built from source is highly recommended on ubuntu 16.04 and later (however, check with `pip show casadi`)
- `pinocchio` latest devel branch (mandatory)
- `pybind11` for generating python bindings
- `urdfdom` (tested on ROS Kinetic)

## Installation 
in order to install casadi-kin-dyn we assume that a system such as conda/mamba has already been installed (for best result especially if it is a plan to use robostack is highly suggested to use [mambaforge](https://github.com/conda-forge/miniforge#mambaforge)). 
first to create the environment for using casadi-kin-dyn and to activate it (assuming that you are in the project folder) you can do
```
mamba env create -f environment.yml
conda activate test_kin_dyn
```
then to install the library assuming that you are inside the casadi_kin_dyn folder you can execute this commands
```
mkdir build 
cd build 
CMAKE_FLAGS="-DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DCMAKE_PREFIX_PATH=$CONDA_PREFIX .."
cmake .. $CMAKE_FLAGS
```

and if the compilation is succesfull you can install it by doing 
```
make install
```

after the installation the library will be not visible under the conda/mamba env 

## Usage (python)
``` python
#!/usr/bin/env python

from casadi import *
import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn

urdf = rospy.get_param('robot_description') # get urdf as string
kindyn = cas_kin_dyn.CasadiKinDyn(urdf) # construct main class

fk_str = kindyn.fk('base_link')
fk = Function.deserialize(fk_str)
print(fk)

id_str = kindyn.rnea()
id = Function.deserialize(id_str)
print(id)
```

Output on a 5-dof system:

```
forward_kinematics:(q[5])->(ee_pos[3],ee_rot[3x3]) SXFunction
rnea:(q[5],v[5],a[5])->(tau[5]) SXFunction
```
