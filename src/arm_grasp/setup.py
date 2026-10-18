from setuptools import setup

package_name = 'arm_grasp'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenzhihuang',
    maintainer_email='rainbowch1@163.com',
    description='Vision-guided grasp state machine for the 6-axis arm.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'grasp_node = arm_grasp.grasp_node:main',
        ],
    },
)
