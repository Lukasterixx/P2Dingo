from setuptools import find_packages, setup

package_name = 'go2_control'

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
    maintainer='lukas',
    maintainer_email='lukas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'walk_forward = go2_control.walk_forward:main',
        'walk_forward_bt = go2_control.behaviors.walk_forward_bt:main',
        'walk_forward_bt_with_permission = go2_control.behaviors.walk_forward_bt_with_permission:main',
        'walk_forward_bt_parallel = go2_control.behaviors.walkBT:main',
    ],
},

)
