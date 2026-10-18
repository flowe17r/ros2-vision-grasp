from setuptools import find_packages, setup

package_name = 'arm_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenzhihuang',
    maintainer_email='rainbowch1@163.com',
    description='HSV segmentation and colored-cube detection nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hsv_segment_node = arm_perception.hsv_segment_node:main',
            'box_detect_node = arm_perception.box_detect_node:main',
            'backproject_node = arm_perception.backproject_node:main',
        ],
    },
)
