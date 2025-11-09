#!/usr/bin/env python3
"""
Standard Python package setup for pip installation.

Install in development mode:
    pip install -e .

This allows the package to be imported from anywhere while
still being editable in place.
"""

from setuptools import setup, find_packages
import os

# Read requirements
def read_requirements():
    req_path = os.path.join(os.path.dirname(__file__), 'requirements.txt')
    if os.path.exists(req_path):
        with open(req_path) as f:
            return [line.strip() for line in f if line.strip() and not line.startswith('#')]
    return []

# Read README for long description
def read_readme():
    readme_path = os.path.join(os.path.dirname(__file__), 'README.md')
    if os.path.exists(readme_path):
        with open(readme_path, encoding='utf-8') as f:
            return f.read()
    return ''

setup(
    name='pybullet_fleet',
    version='0.1.0',
    description='General-purpose PyBullet simulation library for multi-robot fleets',
    long_description=read_readme(),
    long_description_content_type='text/markdown',
    author='Rapyuta Robotics',
    author_email='',
    url='https://github.com/rapyuta-robotics/PyBulletFleet',
    
    # Package discovery - now at top level
    packages=find_packages(include=['pybullet_fleet', 'pybullet_fleet.*']),
    
    # Dependencies
    install_requires=read_requirements(),
    python_requires='>=3.6',
    
    # Additional package data
    include_package_data=True,
    
    # Classifiers
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
    ],
    
    # Entry points (if needed in the future)
    # entry_points={
    #     'console_scripts': [
    #         'pybullet-fleet=pybullet_fleet.cli:main',
    #     ],
    # },
)
