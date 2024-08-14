## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['forcetorquecontrol'],
    package_dir={'': 'src'},
    requires=['std_msgs', 'rospy', 'kinova_msgs', 'kinova_driver', 
              'geometry_msgs', 'armpy','sensor_msgs', 'tf2_msgs',
              'tf2_geometry_msgs','visualization_msgs']
)

setup(**setup_args)
