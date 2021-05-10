import os
import re
from setuptools import setup, find_packages


def readme():
    with open('README.md') as f:
        return f.read()


with open(os.path.join(os.path.dirname(__file__), 'pycollisionavoidance', '__init__.py')) as f:
    version = re.search("__version__ = '([^']+)'", f.read()).group(1)


with open('requirements.txt') as reqs_f:
    reqs = reqs_f.read().strip().split('\n')


setup(
    name='pycollisionavoidance',
    version=version,
    description='collision avoidance Service',
    url='https://github.com/virtual-origami/pypersonnelloc',
    long_description=readme(),
    long_description_content_type='text/markdown',
    author='Karthik Shenoy Panambur, Shantanoo Desai',
    author_email='she@biba.uni-bremen.de, des@biba.uni-bremen.de',
    license='MIT License',
    packages=find_packages(),
    scripts=['bin/collision-avoidance'],
    install_requires=reqs,
    include_data_package=True,
    zip_safe=False
)
