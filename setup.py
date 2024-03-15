from setuptools import setup

package_name = 'odom_imu_fusion'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kis',
    maintainer_email='kis@wj-rdc.jp',
    description='A package for fusing odometry and IMU data to improve localization.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_imu_fusion_node = odom_imu_fusion.odom_imu_fusion:main'
        ],
    },
)
