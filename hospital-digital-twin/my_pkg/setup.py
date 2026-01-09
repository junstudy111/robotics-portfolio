from setuptools import find_packages, setup

package_name = 'my_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav_to_pose = my_pkg.nav_to_pose:main',
            'nav_goThrough = my_pkg.goThrough:main',
            'aruco_detector = my_pkg.aruco_detecto:main',
            'task_manager = my_pkg.task_manager:main',
            'task_manager_node = my_pkg.task_manager_node:main',
            'aruco_detector_pick_grip = my_pkg.aruco_detector_pick_grip:main',
            'nav_executor_node = my_pkg.nav_executor_node:main',
            'init_pos = my_pkg.init_pos:main',
            "arm_executor_node = my_pkg.arm_executor_node:main",
            "aruco_pick_executor_node = my_pkg.aruco_pick_executor_node:main",
            'simple_sequencer = my_pkg.sequencer_node:main',
            'scenario_runner = my_pkg.scenario_runner:main',
        ],
    },
)
