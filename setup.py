# pylint: disable=invalid-name, exec-used
"""Setup russound package."""
from __future__ import absolute_import
import sys
import os
from setuptools import setup, find_packages
# import subprocess
sys.path.insert(0, '.')

CURRENT_DIR = os.path.dirname(__file__)

# to deploy to pip, please use
# make pythonpack
# python setup.py register sdist upload
# and be sure to test it firstly using "python setup.py register sdist upload -r pypitest"
setup(name='XAPX00',
      version='0.1.0',
      description='Control an XAP800/XAP400 unit',
      long_description=open(os.path.join(CURRENT_DIR, 'README.md')).read(),
      install_requires=[],
      license='MIT',
      keywords='XAP800'
      maintainer='Jay Love',
      maintainer_email='jsloveiv@gmail.com',
      zip_safe=True,
      packages=find_packages(),
      include_package_data=True,
      url='https://github.com/jslove/XAPX00.git')
      
