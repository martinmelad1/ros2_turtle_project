from setuptools import setup

package_name = 'arl_ros2_project'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/full_turtle_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martin',
    maintainer_email='martin@example.com',
    description='Turtle drawing project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_commander = arl_ros2_project.turtle_commander:main',
            'shape_node = arl_ros2_project.shape_node:main',
        ],
    },
)
