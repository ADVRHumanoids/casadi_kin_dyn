import os
import sys

import conda.gateways.logging
from conda.cli.main_list import list_packages
from conda.common.compat import text_type

for elem in sys.path:

    exitcode, output = list_packages(elem)
    print('\n'.join(map(text_type, output)))

print('sys.path:', sys.path)


print(os.environ['CONDA_DEFAULT_ENV'])

from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn

urdf = open('cart_pole.urdf', 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)
