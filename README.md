# casadi_kin_dyn
Package for generation of symbolic (SX) expressions of robot kinematics and dynamics. Based on URDF and Pinocchio.

## Dependencies
- `casadi` 3.4.5, built from source is highly recommended on ubuntu 16.04 and later
- `pinocchio` latest devel branch (mandatory)
- `pybind11` for generating python bindings
- `urdfdom` (tested on ROS Kinetic)

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
