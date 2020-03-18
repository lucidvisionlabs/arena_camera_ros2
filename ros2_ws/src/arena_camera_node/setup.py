import os
import shutil
import subprocess
import sys
from glob import glob
from pathlib import Path

from setuptools import setup

package_name = 'arena_camera_node'

##############
'''
def filter_arena_api_files(files):

    arena_api_excluded_dirs = ['__pycache__']
    arena_api_excluded_files = []

    files_filtered = []
    for file_ in files:
        if file_.parent.name in arena_api_excluded_dirs:
            continue
        elif file_.name in arena_api_excluded_files:
            continue
        else:
            files_filtered.append(file_)

    #files_filtered_str = [str(_) for _ in list(files_filtered)]
    # for _ in files_filtered_str:
    #    print(_, ' --> ', Path(_).resolve())

    return files_filtered

def arena_api_files():

    # installation path
    python_version_name = Path(sys.executable).resolve().name
    arena_api_installation_path = Path(sys.executable).parent.parent / \
        f'lib/{python_version_name}/site-packages/arena_api'

    print('## ', arena_api_installation_path)

    # if not arena_api_installation_path.exists():
    #    raise Exception(
    #        'arena_camera_node depends on arena_api package. '
    #        'Please install arena_api in your pip')

    # change to relative to this file (if possible)

    root_relative_to_cwd = Path(os.getcwd())
    # TODO MAKE CROSS PLATFORM and multi root case
    while str(root_relative_to_cwd.resolve()) != '/':
        root_relative_to_cwd /= '..'
    root_relative_to_cwd = root_relative_to_cwd.relative_to(os.getcwd())
    arena_api_relative_to_root = arena_api_installation_path.relative_to('/')
    arena_api_relative_to_cwd = root_relative_to_cwd / arena_api_relative_to_root

    print('## ', arena_api_relative_to_cwd)
    print('## ', '../../../../../../home/abdul/arena_camera_ros2/ros2_ws/ve_dev/lib/python3.6/site-packages/arena_api/*.txt')

    all_files = arena_api_relative_to_cwd.glob('**/*.*')

    files_filtered = filter_arena_api_files(all_files)

    #files_filtered_str = map(str, files_filtered)
    # for _ in files_filtered_str:
    #    print(_, ' --> ', Path(_).resolve())

    return files_filtered
'''
#####################


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

#./install/arena_camera_node/lib/python3.6/site-packages/site.py
#./ve_dev/lib/python3.6/site-packages


def copy_arena_api_files_to_install_dir():
    arena_api_path = get_arena_api_installation_path()
    #print(arena_api_path)
    #print('/home/abdul/arena_camera_ros2/ros2_ws/ve_dev/lib/python3.6/site-packages/arena_api (corr)')

    python_version_name = Path(sys.executable).resolve().name
    '''
    install_path_prefix = Path(__file__).parent.parent.parent
    install_path = install_path_prefix / 'install' / \
        package_name / 'lib' / python_version_name / 'site-packages' / 'arena_api'
    #print(install_path)
    #print('/home/abdul/arena_camera_ros2/ros2_ws/install/arena_camera_node/lib/python3.6/site-packages/arena_api (corr)')
    '''

    # - common root between __file__ and installation path
    # - geting __file__ and it parent would not work as colcon does 
    # something weired to the setup file
    # - starts from the location of this file
    # install_path_prefix = this /home/abdul/arena_camera_ros2/ros2_ws relative to __file__
    install_path_prefix = Path('./../../')
    install_path = install_path_prefix / 'install' / \
        package_name / 'lib' / python_version_name / 'site-packages' / 'arena_api'
    #install_path = install_path.resolve()
    #print(install_path)
    #print('/home/abdul/arena_camera_ros2/ros2_ws/install/arena_camera_node/lib/python3.6/site-packages/arena_api (corr)')



    # remove old if exists
    if install_path.exists():
        shutil.rmtree(install_path)

    shutil.copytree(arena_api_path, install_path, symlinks=True)

#copy_arena_api_files_to_install_dir()

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

        # 4 arena_api package (for now it copys files using a function)

        #,('lib/python3.6/site-packages', [str(copy_arena_api_files_to_install_dir())])
        #,('lib/python3.6/site-packages/', ['NoneHACK'])
        
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
    data_files=get_date_files() ,

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
            'run = arena_camera_node.run:run'
        ],
    },
)
