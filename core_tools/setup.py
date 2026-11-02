from setuptools import find_packages, setup

package_name = 'core_tools'#パッケージ名

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/base_sysid.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='pizac',
    maintainer_email='pizac.twitter@gmail.com',
    description='core_tools',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'base_sysid_probe = core_tools.base_sysid_probe:main',
            'base_sysid_fit = core_tools.base_sysid_fit:main',
            'base_sysid_hold = core_tools.base_sysid_fit:hold_main',
            'motor_tool = core_tools.motor_tool:main',#GUI　プログラム名
        ],
    },
)
