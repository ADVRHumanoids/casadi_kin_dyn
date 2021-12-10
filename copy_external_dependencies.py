#!/usr/bin/env python3

import subprocess
import argparse
import shutil
import os
import sys

import ctypes
import ctypes.util
libdl = ctypes.CDLL(ctypes.util.find_library('dl'))

class Dl_info(ctypes.Structure):
    _fields_ = (('dli_fname', ctypes.c_char_p),
                ('dli_fbase', ctypes.c_void_p),
                ('dli_sname', ctypes.c_char_p),
                ('dli_saddr', ctypes.c_void_p))

libdl.dladdr.argtypes = (ctypes.c_void_p, ctypes.POINTER(Dl_info))

def find_lib(libname):

    sh_obj = ctypes.cdll.LoadLibrary(libname)

    info = Dl_info()
    result = libdl.dladdr(sh_obj._init, ctypes.byref(info))

    if result and info.dli_fname:
        libdl_path = info.dli_fname.decode(sys.getfilesystemencoding())
        return libdl_path
    else:
        raise RuntimeError(f'resource {libname} not found')

def set_rpath(libname):
    subprocess.run(['patchelf', '--remove-rpath', libname])
    subprocess.run(['patchelf', '--force-rpath', '--set-rpath', '$ORIGIN', libname])


parser = argparse.ArgumentParser(description='This tool discovers the location of shared library transitive dependencies')
parser.add_argument('solib', nargs=1, help='path to the input shared library')
parser.add_argument('--dst', '-d', required=False, help='destination path where found libs are copied')
parser.add_argument('--find-solib', '-f', action='store_true', help='use dlopen/dladdr to find the path to the shared library')
parser.add_argument('--copy-solib', '-c', action='store_true', help='copy input shared library as well')

args = parser.parse_args()

solib_path = find_lib(args.solib[0]) if args.find_solib else args.solib[0]
ldd_output = subprocess.check_output(['ldd', solib_path]).decode()

if args.dst:
    os.makedirs(name=args.dst, exist_ok=True)

if args.dst and args.copy_solib:
    dst_file = os.path.join(args.dst, os.path.basename(solib_path))
    shutil.copyfile(src=solib_path, dst=dst_file)
    set_rpath(dst_file)

for line in ldd_output.split('\n'):
    tokens = line.strip().split(' ')
    if '=>' not in tokens:
        continue
    libname = tokens[0]
    path = tokens[2]
    exclude_lib = path.startswith('/usr/lib') or path.startswith('/lib')
    
    if not exclude_lib:
        print(path)
        if args.dst is not None:
            dst_file = os.path.join(args.dst, libname)
            shutil.copyfile(src=path, dst=dst_file)
            set_rpath(dst_file)