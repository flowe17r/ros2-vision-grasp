from setuptools import setup

package_name = 'arm_mcp'

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
    maintainer_email='flowe17r@users.noreply.github.com',
    description='MCP Server exposing the arm interfaces to LLM tool calling.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mcp_server = arm_mcp.mcp_server:main',
        ],
    },
)
