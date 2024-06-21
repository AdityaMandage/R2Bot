from setuptools import setup

package_name = 'autonomy_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Autonomy functionalities for the robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_mover_node = autonomy_package.ball_mover_node:main',
            'move_to_ball_node = autonomy_package.move_to_ball_node:main'
        ],
    },
)
