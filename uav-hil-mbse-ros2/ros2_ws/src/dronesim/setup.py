from setuptools import setup

package_name = 'dronesim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ak',
    maintainer_email='ak@fh-swf.de',
    description='Prototype dronesim package for collision risk under sensor noise',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drone_node = dronesim.drone_node:main',
            'integrated_uav_controller = dronesim.integrated_uav_controller:main',
        ],
    },
)
