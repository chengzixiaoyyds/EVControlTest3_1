from setuptools import find_packages, setup

package_name = 'rov_pid_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chengzixiao',
    maintainer_email='chengzixiaoyyds@qq.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_parameter_publisher = rov_pid_controller.pid_parameter_publisher:main',
            'pid_controller = rov_pid_controller.pid_controller:main',
        ],
    },
)
