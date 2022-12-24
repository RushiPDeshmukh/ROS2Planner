from setuptools import setup

package_name = 'sim_2d'

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
    maintainer='rushi',
    maintainer_email='rdeshmukh@wpi.edu',
    description='This package contains pygame 2D simulator node, various planner nodes, car node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "maze_simulator = sim_2d.maze_simulator:main",
            "maze_generator = sim_2d.maze_generator:main",
            "player = sim_2d.player:main",
            "planner = sim_2d.planner:main"
        ],
    },
)
