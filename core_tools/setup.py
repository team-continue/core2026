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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pizac',
    maintainer_email='pizac.twitter@gmail.com',
    description='core_tools',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'motor_tool = core_tools.motor_tool:main',#GUI　プログラム名
        ],
    },
)
