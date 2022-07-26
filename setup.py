from setuptools import setup, find_packages

setup(
    name='kicker',
    version='0.0.1',    
    packages=find_packages(include=['kicker', 'kicker.*']),
    package_data={'kicker': ['assets/*.urdf']},
    url='',
    license='',
    author='LDRL',
    author_email='',
    description='Kicker package',
    long_description="""
        Kicker environment for RL
    """,
    include_package_data=True,
    install_requires=open("requirements.txt").read().split("\n"),
)
