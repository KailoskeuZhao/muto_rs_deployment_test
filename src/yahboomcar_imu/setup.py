from setuptools import setup

package_name = 'yahboomcar_imu'

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
    maintainer='nx-ros2',
    maintainer_email='1461190907@qq.com',
    description='IMU publisher for Yahboom car Muto controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yahboomcar_imu = yahboomcar_imu.imu_node:main',
            'gyro_orientation_test_node = yahboomcar_imu.gyro_orientation_test_node:main',
        ],
    },
)
