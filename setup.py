from setuptools import setup, find_packages

setup(
    # name='Kicker-in-the-Gym',
    name="kicker",
    version='0.0.3',
    package_dir={'kicker': 'src/kicker'},
    # packages=find_packages("./src", include=['kicker', 'kicker.*']),
    packages=["kicker", "kicker.gym_env"],
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
