from setuptools import setup, find_packages

setup(
    name='pybop_lib',
    version='1.0',
    packages=find_packages(exclude=('docs')),
    author='PM van der Burg',
    author_email='pmvanderburg@tudelft.nl',
    license='MIT license',
    package_data={'pybop_lib':['*']},
)
