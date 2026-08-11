from setuptools import find_packages, setup

package_name = 'cerulean_surveyor_mapping'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pointcloud_georeferencer.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='John Walsh',
    maintainer_email='john.walsh@whoi.edu',
    description='Georeferenced pointcloud and bathymetry mapping built on top of the Cerulean Surveyor 240 MBES driver',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pointcloud_georeferencer_node = cerulean_surveyor_mapping.pointcloud_georeferencer_node:main',
        ],
    },
)
