from setuptools import find_packages, setup

package_name = 'g1_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='2010040054ece@gmail.com',
    description='Gesture control and joint tests for UG1 robot',
    license='MIT',  
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Test publisher
            'g1_joint_test_pub = g1_control.g1_joint_test_pub:main',
            
            # Parameterized gesture node
            'gestures_node = g1_control.gestures_control:main',
            
            # Individual gesture scripts
            'wave_gesture = g1_control.wave_gesture:main',
            'point_gesture = g1_control.point_gesture:main',
            'grasp_gesture = g1_control.grasp_gesture:main',
            'object_localizer = g1_control.object_localizer:main'
        ],
    },
)
