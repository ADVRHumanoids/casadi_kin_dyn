import os
import sys
print('sys.path:', sys.path)
print(os.environ['CONDA_DEFAULT_ENV'])

from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn

urdf = open('cart_pole.urdf', 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)
