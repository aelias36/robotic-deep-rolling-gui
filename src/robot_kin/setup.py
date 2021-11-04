from setuptools import setup
from distutils.core import Extension, setup
from Cython.Build import cythonize

# ext = Extension(name="hello", sources=["helloworld.pyx"])
ext = Extension(name="abb_6640_kinematics", sources=["abb_6640_kinematics.pyx"])
setup(ext_modules=cythonize(ext))