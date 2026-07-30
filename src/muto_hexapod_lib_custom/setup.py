from setuptools import find_packages, setup


package_name = 'muto_hexapod_lib_custom'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'LICENSE', 'NOTICE']),
    ],
    install_requires=['setuptools', 'pyserial'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='nx-ros2@todo.todo',
    description=(
        'ROS-packaged Muto serial control and commanded gait generation'
    ),
    license='MIT',
)
