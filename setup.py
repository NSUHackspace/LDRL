from setuptools import setup, find_packages

setup(
    name='kicker',
    version='0.0.3',
    package_dir={'kicker': 'src/kicker'},
    # packages=find_packages("./src", include=['kicker', 'kicker.*']),
    packages=["kicker", "kicker.ai", "kicker.gym_env", "kicker.is_done_functions", "kicker.reset_functions", "kicker.reward_functions", "kicker.scene", "kicker.utils"],
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
