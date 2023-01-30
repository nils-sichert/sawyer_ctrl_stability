from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["sawyer_ctrl_stability"],
    package_dir={"": "scripts"},
)

setup(**setup_args)