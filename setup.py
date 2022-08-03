from setuptools import setup, find_packages

setup(
    name='Kicker-in-the-Gym',
    version='',
    package_dir={'kicker': 'src/kicker'},
    packages=find_packages("./src", include=['kicker', 'kicker.*']),
    package_data={'kicker': ['assets/*.urdf']},
    url='',
    license='',
    author='LDRL',
    author_email='',
    description='',
    include_package_data=True
)
