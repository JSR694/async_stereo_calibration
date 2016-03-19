## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['async_stereo_calibration'],
    package_dir={'src': 'src', 'srv':'srv'},
    requires=['std_msgs', 'rospy','rviz','sensor_msgs','geometry_msgs','tf','cv_bridge','apriltags_ros'] 
)

setup(**setup_args)

