import warnings
warnings.filterwarnings("ignore")

import os
from glob import glob
from setuptools import setup

package_name = 'ex2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*_launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eisuke',
    maintainer_email='eisuke@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "talker=ex2.talker:main",
        "listener=ex2.listener:main",
        ],
    },
)
