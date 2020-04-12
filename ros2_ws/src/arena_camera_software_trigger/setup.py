import os
import sys
from glob import glob

from setuptools import setup

package_name = 'arena_camera_software_trigger'


def get_date_files():

    return [

        # 1 TODO investigate this
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # 2 package.xml
        (os.path.join('share', package_name), ['package.xml']),

        # 3 launch files
        (os.path.join('share', package_name), glob('launch/*'))

        # 4 arena_api package (for now it copies files using a function)

        # ('lib/python3.6/site-packages__2', glob('../../../../*.txt', recursive=True) # works as home
        # ('lib/python3.6/site-packages__2', glob('../../../../../../*.txt', recursive=True) # works as root
        # ('lib/python3.6/site-packages__2',
        # glob('../../../../../../home/abdul/arena_camera_ros2/ros2_ws/ve_dev/lib/python3.6/site-packages/arena_api/*', recursive=True)) # work with _colcon
        #('lib/python3.6/site-packages__2', hard_arena_api_paths())
    ]


setup(
    name=package_name,

    version='0.0.0',

    # Packages to export
    packages=[package_name],

    # Files we want to install, specifically launch files
    data_files=get_date_files(),

    install_requires=['setuptools'],
    #install_requires=['setuptools', 'arena_api'],

    zip_safe=True,

    author='LUCID Vision Labs',

    author_email='support@thinklucid.com',

    maintainer='LUCID Vision Labs',

    maintainer_email='support@thinklucid.com',

    keywords=['ROS'],

    classifiers=[  # TODO TALK WITH GARA
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],

    description='Lucid Camera ROS2 Software Trigger Driver',

    license='Apache License, Version 2.0',  # TODO TALK WITH GARA

    tests_require=['pytest'],

    #entry_points={
    #    'console_scripts': [
    #        'trigger_image = arena_camera_software_trigger.trigger_image:trigger_image_fn',
    #    ],
    #},
)
