import os
import shutil
import subprocess
import sys
from glob import glob
from pathlib import Path

from setuptools import setup

package_name = 'arena_camera_node'


def get_arena_api_installation_path():
    try:
        pip_show = subprocess.check_output(['pip', "show", "arena_api"],
                                           encoding='utf-8')
    except:
        try:
            pip_show = subprocess.check_output(['pip3', "show", "arena_api"],
                                               encoding='utf-8')
        except:
            raise Exception('pip or arena_api is not installed. '
                            'please install pip and arena_api')

    lines = pip_show.split('\n')
    for line in lines:
        line = line.split(': ')  # space is intentional
        if line[0] == 'Location':
            path = line[1]
            break
    else:
        raise Exception(f'arena_api does not have a location info. Please '
                        f'reinstall arena_api')

    return Path(path) / 'arena_api'


def copy_arena_api_files_to_install_dir():
    arena_api_path = get_arena_api_installation_path()

    python_version_name = Path(sys.executable).resolve().name

    # - common root between __file__ and installation path
    # - geting __file__ and it parent would not work as colcon does
    # something weired to the setup file
    # - starts from the location of this file
    # install_path_prefix = this /home/abdul/arena_camera_ros2/ros2_ws relative to __file__
    install_path_prefix = Path('./../../')
    install_path = install_path_prefix / 'install' / \
        package_name / 'lib' / python_version_name / 'site-packages' / 'arena_api'

    # remove old if exists
    if install_path.exists():
        shutil.rmtree(install_path)

    shutil.copytree(arena_api_path, install_path, symlinks=True)

# copy_arena_api_files_to_install_dir()


def get_date_files():
    # a hack to add the arena_api files to arena install
    copy_arena_api_files_to_install_dir()

    return [

        # 1 TODO investigate this
        # ('share/ament_index/resource_index/packages',
        # ['resource/' + package_name]),

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

    description='Lucid Camera ROS2 Driver',

    license='Apache License, Version 2.0',  # TODO TALK WITH GARA

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'start = arena_camera_node.main:main',
            'trigger_image = arena_camera_node.trigger_image_client:main',
        ],
    },
)
