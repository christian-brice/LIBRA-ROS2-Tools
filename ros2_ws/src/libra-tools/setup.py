from setuptools import find_packages, setup

package_name = 'libra-tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christian Brice',
    maintainer_email='brice.c.67b9@m.isct.ac.jp',
    description='Supplemental ROS2 tools for the LIBRA Project',
    license='MIT',
    entry_points={
        'console_scripts': [
            # (format: 'executable_name = package_name.path_to_script:main_function')
            'tip_path_publisher = libra-tools.tip_path_publisher:main',
        ],
    },
)
