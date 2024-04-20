from setuptools import find_packages, setup
import glob 
import os 

package_name = 'best_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
        glob.glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/param',
        glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkyu',
    maintainer_email='dknjy3313@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_publisher = best_camera.img_publisher:main',
            'img_control = best_camera.img_control:main',
            'cartoon = best_camera.cartoon:main',
            'laplace = best_camera.laplace:main',
            'edge = best_camera.edge:main',
            'optical_flow = best_camera.optical_flow:main',
            'gray = best_camera.gray:main',
            'camera_node = best_camera.camera_node:main',
            'picture_node = best_camera.picture_node:main',
            'aruco = best_camera.aruco:main',
        ],
    },
)
