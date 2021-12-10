import setuptools
import subprocess

setuptools.setup(
    name="casadi_kin_dyn",
    version="1.0.0",
    author="Francesco Ruscelli",
    author_email="francesco.ruscelli@iit.it",
    description="Library for Trajectory Optimization based on CasADi",
    # long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/FrancescoRuscelli/horizon_gui",
    # project_urls={
    #     "Bug Tracker": "no",
    # },
    # classifiers=[
    #     "Programming Language :: Python :: 3",
    #     "Operating System :: Ubuntu",
    # ],
    # package_dir={"": "horizon"},
    packages=['casadi_kin_dyn'],
    install_requires=[],
    # packages=setuptools.find_packages(),
    python_requires=">=3.6"
    
)
