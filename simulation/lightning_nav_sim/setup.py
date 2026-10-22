from glob import glob
from setuptools import setup

setup(
    name='lightning_nav_sim', version='0.1.0', packages=['lightning_nav_sim'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/lightning_nav_sim']),
        ('share/lightning_nav_sim', ['package.xml']),
        ('share/lightning_nav_sim/launch', glob('launch/*.py')),
        ('share/lightning_nav_sim/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'], zip_safe=True,
    maintainer='lightning-lm contributors',
    maintainer_email='tianqingye12-bit@users.noreply.github.com',
    description='Independent-world Nav2 simulation and raw/clean map evaluation',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'nav_sim_maps = lightning_nav_sim.maps:main',
        'nav_sim_world = lightning_nav_sim.ros_sim:main',
        'nav_sim_trial = lightning_nav_sim.trial:main',
        'nav_sim_compare = lightning_nav_sim.compare:main',
    ]},
)
