from setuptools import find_packages, setup

package_name = 'morpheus_teleop'

setup(
 name=package_name,
 version='2.0.0',
 packages=find_packages(exclude=['test']),
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Brian Sanyu Huo',
 maintainer_email='bshuo@ucdavis.edu',
 description='The morpheus_teleop package',
 license='GPL-3.0-only',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'teleop_twist = morpheus_teleop.teleop_twist:main'
     ],
   },
)