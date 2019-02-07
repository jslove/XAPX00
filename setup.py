# pylint: disable=invalid-name, exec-used
"""Setup XAPX00 package."""
from __future__ import absolute_import
import sys
import os
from setuptools import setup, find_packages
# import subprocess
sys.path.insert(0, '.')

CURRENT_DIR = os.path.dirname(__file__)


def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

# to deploy to pip, please use
# make pythonpack
# python setup.py register sdist upload
# and be sure to test it firstly using "python setup.py register sdist upload -r pypitest"

setup(name='XAPX00',
      version='0.2.7',
      description='Control an XAP800/XAP400 unit',
      long_description=read('README.md'),
      install_requires=['pyserial>=3.3'],
      license='GNU',
      keywords=['XAP800', 'XAPX00', 'XAP400', 'Genternet', 'ClearOne'],
      maintainer='Jay Love',
      maintainer_email='jsloveiv@gmail.com',
      zip_safe=True,
      packages=find_packages(),
      include_package_data=True,
      url='https://github.com/jslove/XAPX00.git')

