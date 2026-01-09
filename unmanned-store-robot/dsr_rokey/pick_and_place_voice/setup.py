from setuptools import find_packages, setup
import glob
import os

package_name = 'pick_and_place_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[
        'robot_control*',
        'voice_processing*', 
        'object_detection*',
        'web*'
    ]),

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob.glob('resource/*')),
        ('share/' + package_name + '/resource', glob.glob('resource/.env')),
        
        # ⭐ [추가됨] Launch 파일 설치 경로 설정
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey4090',
    maintainer_email='rokey4090@todo.todo',
    description='Pick and Place with Voice Control',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # 1. Robot Control Actions
            'bring_action_server = robot_control.bring_action_server:main',
            'clear_action_server = robot_control.clear_action_server:main',
            'purchase_action_server = robot_control.purchase_action_server:main',
            'return_action_server = robot_control.return_action_server:main',
            
            # 2. Hardware & Service
            'arduino_control_node = robot_control.arduino_control_node:main',
            'segmentation_service = object_detection.segmentation_service:main',
            
            # 3. Voice Processing
            'get_keyword = voice_processing.get_keyword:main',
            
            # 4. Web Interface
            'fastapi_server = web.fastapi_server:main',
            
            # 5. Others (Legacy or Utils)
            'object_detection = object_detection.detection:main',
            'object_seg = object_detection.segmentation:main',
            'robot_control = robot_control.robot_control:main',
        ],
    },
)