from setuptools import find_packages, setup

package_name = 'turtlesim_catch_them_all_py'

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
    maintainer='elena',
    maintainer_email='evillalba001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'turtle_controller = turtlesim_catch_them_all_py.turtle_controller:main',
            'turtle_spawner = turtlesim_catch_them_all_py.turtle_spawner:main',

        ],
    },
)
