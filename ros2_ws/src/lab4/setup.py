from setuptools import find_packages, setup

package_name = 'lab4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup_and_waypoints.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clanker',
    maintainer_email='pm674@cornell.edu',
    description='Social Navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'social_navigation_node = lab4.social_navigation_node:main'
        ],
    },
)
