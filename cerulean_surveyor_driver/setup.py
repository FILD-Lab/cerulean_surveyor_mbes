from setuptools import find_packages, setup

package_name = 'cerulean_surveyor_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/surveyor_mbes_driver.yaml']),
        ('share/' + package_name + '/launch', ['launch/surveyor_mbes_driver.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tony',
    maintainer_email='tony.jacob@uri.edu',
    description='ROS2 driver for the Cerulean Surveyor 240 MBES',
    license='MIT',
    entry_points={
        'console_scripts': [
            'surveyor_mbes_driver = cerulean_surveyor_driver.surveyor_mbes_driver:main',
            'surveyor_depth_node = cerulean_surveyor_driver.surveyor_depth_node:main',
        ],
    },
)
