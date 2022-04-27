import setuptools

setuptools.setup(
    name="casadi_kin_dyn",
    version="1.3.2",
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
