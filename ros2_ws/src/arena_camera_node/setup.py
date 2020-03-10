from setuptools import setup

package_name = 'arena_camera_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
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
            'main = arena_camera_node.main:main'
        ],
    },
)
