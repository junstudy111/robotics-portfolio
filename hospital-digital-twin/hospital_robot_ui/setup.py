from setuptools import find_packages, setup

package_name = 'hospital_robot_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    data_files=[
        # ROS 패키지 인덱스 등록
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml 설치
        ('share/' + package_name, ['package.xml']),

        # 정적 파일 (FastAPI)
        ('share/' + package_name + '/static', [
            'hospital_robot/static/app.js',
            'hospital_robot/static/style.css',
        ]),

        # 템플릿 파일
        ('share/' + package_name + '/templates', [
            'hospital_robot/templates/index.html',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='Hospital UI ROS2 node',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 🔴 Python 모듈 이름은 그대로 hospital_robot
            'hospital_ui_node = hospital_robot.hospital_ui_node:main',
        ],
    },
)
