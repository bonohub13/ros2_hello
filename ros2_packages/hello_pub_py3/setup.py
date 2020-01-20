from setuptools import setup

package_name = 'hello_pub_py3'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kensuke Saito',
    maintainer_email='ken.saito.0813@gmail.com',
    description='\"Hello World!\" in ROS2 (publisher only)',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_pub = hello_pub_py3.hello_pub:main'
        ],
    },
)
