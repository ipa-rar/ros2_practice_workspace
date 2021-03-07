from setuptools import setup

package_name = 'ros2_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ragesh_ramachandran',
    maintainer_email='ragesh.ramachandran@ipa.fraunhofer.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'number_publisher = ros2_pub.main:main'
            #name = pkg_name.file_name:ros_fun_name
        ],
    },
)
