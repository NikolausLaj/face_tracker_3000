from setuptools import find_packages, setup

package_name = 'face_tracker_3000'

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
    maintainer='ubuntu',
    maintainer_email='nikolaus.lajtai@gmx.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_offset = face_tracker_3000.face_offset:main',
            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
)
