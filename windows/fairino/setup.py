# setup.py
# python3 setup.py build_ext --inplace
# python setup.py build_ext --inplace                   (python3.12之前的使用)
# python setup.py build_ext --inplace  --compiler=msvc  (python3.12使用)
from distutils.core import setup                   #  (python3.12之前的使用)
# from setuptools import setup                         #  (python3.12使用)
from Cython.Build import cythonize
setup(name='Robot', ext_modules=cythonize('Robot.py'))
