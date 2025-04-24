from setuptools import find_packages, setup

package_name = 'teleop_wamv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['teleop_wamv/launch/system_bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimminjung',
    maintainer_email='kmjeong000@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_publisher = teleop_wamv.keyboard_publisher:main',
            'auto_thrust_publisher = teleop_wamv.auto_thrust_publisher:main',
            'image_saver = teleop_wamv.image_saver:main',
            'base64_pub = teleop_wamv.base64_pub:main',
            'gpt_bridge = teleop_wamv.gpt_bridge:main',
            'gpt_description = teleop_wamv.gpt_description:main',
            
        ],
    },
)
