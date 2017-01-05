'''
Created on 2.11.2016

@author: Samuli Rahkonen
'''

from setuptools import setup, find_packages

with open('README.md') as f:
    readme = f.read()

with open('LICENSE') as f:
    license_file = f.read()

install_requires = ['pyserial>=3.2.1']

setup(name='scx11scanner',
      version='0.0.1',
      description=readme,
      author='Samuli Rahkonen',
      author_email='samuli.rahkonen@jyu.fi',
      url='https://yousource.it.jyu.fi/hsipython/scx11-scanner',
      install_requires=install_requires,
      packages=find_packages(),
      license=license_file
      )
