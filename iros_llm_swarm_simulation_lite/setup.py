from setuptools import find_packages, setup
from glob import glob
package_name = 'iros_llm_swarm_simulation_lite'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/stage_sim', glob('world/*.world')),
        ('share/' + package_name + '/stage_sim', glob('map/*')),
        ('share/' + package_name + '/scenario', glob('scenario/*')),
        ('share/' + package_name + '/map_descriptions', glob('map_descriptions/*.yaml')),
        ('share/' + package_name + '/stage_sim', glob('robot_description/*.inc')),
        ('share/' + package_name + '/robot_description', glob('robot_description/*.urdf.xacro')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabian',
    maintainer_email='fabian@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
