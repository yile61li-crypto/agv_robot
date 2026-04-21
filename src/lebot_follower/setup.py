from setuptools import find_packages, setup

package_name = 'lebot_follower'

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
    maintainer='lyl',
    maintainer_email='q19864919467@163.com',
    description='LeBot follower controller and leader pose publisher',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'entity_pose_publisher=lebot_follower.entity_pose_publisher:main',
            'follower_controller=lebot_follower.follower_controller:main',
        ],
    },
)
