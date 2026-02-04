from setuptools import find_packages, setup

package_name = 'nodes_topics_demo'

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
    maintainer='sns',
    maintainer_email='syednazmussakib64@gmail.com',
    description='Demo package to learn ROS 2 Nodes and Topics',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Format: 'executable_name = package.module:function'
            'number_publisher = nodes_topics_demo.number_publisher:main',
            'number_subscriber = nodes_topics_demo.number_subscriber:main',
        ],
    },
)
