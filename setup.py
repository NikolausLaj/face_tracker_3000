from setuptools import setup

package_name = 'face_tracker_3000'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Face Tracker 3000 ROS2 Python Package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'face_tracker_3000_node = face_tracker_3000.face_tracker_3000_node:main',
        ],
    },
)
