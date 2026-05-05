import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'iros_llm_orchestrator'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'prompts'), glob('prompts/*.txt')),
        (os.path.join('share', package_name, 'prompts', 'maps'), glob('prompts/maps/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='LLM orchestrator for swarm behavior tree',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'decision_server  = iros_llm_orchestrator.decision_server:main',
            'passive_observer = iros_llm_orchestrator.passive_observer:main',
            'user_chat        = iros_llm_orchestrator.user_chat_node:main',
            'chat_server      = iros_llm_orchestrator.chat_server:main',
        ],
    },
)
