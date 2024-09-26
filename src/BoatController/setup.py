from setuptools import find_packages, setup
package_name = 'BoatController'


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
    maintainer='dinesh',
    maintainer_email='dskumar@ucdavis.edu',
    description='TODO: Follow Readme',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "thrust_subscriber = BoatController.thrust_subscriber:main",
            "thrust_publisher = BoatController.thrust_publisher:main",
            "position_publisher = BoatController.position_publisher:main",
            "position_subscriber = BoatController.position_subscriber:main",
            "velocityYaw_publisher = BoatController.velocityYaw_publisher:main",
            "velocityYaw_subscriber = BoatController.velocityYaw_subscriber:main",
        ],
    },
)
