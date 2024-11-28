# setup.py
# python3 setup.py build_ext --inplace
# python setup.py build_ext --inplace
from distutils.core import setup
from Cython.Build import cythonize
setup(name='Robot', ext_modules=cythonize('Robot.py'))
