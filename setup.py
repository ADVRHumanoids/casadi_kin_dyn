import setuptools
import os
import codecs

def read(rel_path):
    here = os.path.abspath(os.path.dirname(__file__))
    with codecs.open(os.path.join(here, rel_path), 'r') as fp:
        return fp.read()

setuptools.setup(
    name="casadi_kin_dyn",
    version=read("version.yml"),
    author="Arturo Laurenzi",
    author_email="arturo.laurenzi@iit.it",
    description="Package for generation of symbolic (casadi::SX) expressions of robot kinematics and dynamics. Based on URDF and Pinocchio.",
    long_description_content_type="text/markdown",
    url="https://github.com/ADVRHumanoids/casadi_kin_dyn",
    packages=['casadi_kin_dyn'],
    install_requires=[],
    python_requires=">=3.6",
    ext_modules=[
        setuptools.Extension(
            name='pycasadi_kin_din',
            sources=[]
        )
    ]
)
