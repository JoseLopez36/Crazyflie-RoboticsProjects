import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'crazyflie_robotics_projects_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*yaml'))
    ],
    install_requires=['setuptools', 'pynput'],
    zip_safe=True,
    maintainer='Jorge Prieto Álvarez, Víctor Javier Granero Gil, José Francisco López Ruiz',
    maintainer_email='josloprui6@alum.us.es',
    description='Main package for the Crazyflie Robotics Projects',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'coordinator_node = crazyflie_robotics_projects_pkg.coordinator_node:main',
            'darp_node = crazyflie_robotics_projects_pkg.darp_node:main',
            'planning_node = crazyflie_robotics_projects_pkg.planning_node:main',
            'control_node = crazyflie_robotics_projects_pkg.control_node:main',
            'visualization_node = crazyflie_robotics_projects_pkg.visualization_node:main',
            'test_node = crazyflie_robotics_projects_pkg.test_node:main'
        ],
    },
)