from setuptools import setup

package_name = 'influxdb_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'min_distance_sub_inDB = influxdb_test.min_distance_sub_inDB:main',
            
            'nuitrack_to_InfluxDB = influxdb_test.nuitrack_to_InfluxDB:main',
            'heatmap_test = influxdb_test.heatmap_test:main'
        ],
    },
)
