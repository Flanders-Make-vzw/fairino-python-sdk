# setup.py
# python3 setup.py build_ext --inplace
# python setup.py build_ext --inplace                   (python3.12之前的使用)
# python setup.py build_ext --inplace  --compiler=msvc  (python3.12使用)
# from distutils.core import setup                   #  (python3.12之前的使用)
from setuptools import setup, find_packages, Extension                         #  (python3.12使用)
from Cython.Build import cythonize
import os

# Get the current directory (linux/fairino)
current_dir = os.path.dirname(os.path.abspath(__file__))

# Define the Robot extension module
robot_extension = Extension(
    'fairino.Robot',
    sources=['Robot.py'],
    language='c'
)

setup(
    name='fairino-python-sdk',
    version='1.0.0',
    description='Fairino Robot Python SDK',
    packages=['fairino'],
    package_dir={'fairino': '.'},
    ext_modules=cythonize([robot_extension]),
    install_requires=[
        'cython',
    ],
    python_requires='>=3.8',
    zip_safe=False,
)
