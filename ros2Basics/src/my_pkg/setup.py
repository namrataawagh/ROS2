from setuptools import find_packages, setup

package_name = 'my_pkg'

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
    maintainer='namrata',
    maintainer_email='namrata@todo.todo',
    description='My custom ROS 2 Python package',
    license='MIT',  
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_pkg.my_node:main',  # This means: call main() from my_node.py
        ],
    },
)
