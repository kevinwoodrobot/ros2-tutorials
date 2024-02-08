from setuptools import find_packages, setup
# ADDED 
import os
from glob import glob
# END ADDED

package_name = 'learning_tf2_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
# ADDED
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
# END ADDED

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevinwoodrobot',
    maintainer_email='kevinwoodrobotics@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_turtle_tf2_broadcaster = learning_tf2_py.static_turtle_tf2_broadcaster:main',
# ADDED
            'turtle_tf2_broadcaster = learning_tf2_py.turtle_tf2_broadcaster:main',
# END ADDED
        ],
    },
)
