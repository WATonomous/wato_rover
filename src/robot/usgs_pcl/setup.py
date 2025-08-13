from setuptools import setup

package_name = 'usgs_pcl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/usgs_pcl.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eddy Zhou',
    maintainer_email='e23zhou@watonomous.ca',
    description='Convert USGS .las point cloud data to ROS 2 PointCloud2 messages',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usgs_pcl = usgs_pcl.usgs_pcl:main'
        ],
    },
)
