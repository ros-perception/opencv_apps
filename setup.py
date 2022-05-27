import catkin_pkg
from distutils.version import LooseVersion
if LooseVersion(catkin_pkg.__version__) <= LooseVersion('0.4.8'):
    # on hydro we need to use distutils
    from distutils.core import setup
else:
    from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['opencv_apps'],
    package_dir={'': 'src'},
)
setup(**d)
