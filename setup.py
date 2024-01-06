from setuptools import find_packages, setup

package_name = 'px4_offboard'

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
    maintainer='gspark',
    maintainer_email='pks87@nate.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mc_control = px4_offboard.mc_control:main',
            'fw_control = px4_offboard.fw_control:main',
        ],
    },
)
