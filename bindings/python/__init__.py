import sys
if sys.version_info[0] == 3:
    import casadi_kin_dyn.py3casadi_kin_dyn as pycasadi_kin_dyn
else:
    import casadi_kin_dyn.pycasadi_kin_dyn as pycasadi_kin_dyn
