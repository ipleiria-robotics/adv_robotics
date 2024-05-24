from setuptools import setup
from glob import glob

package_name = 'tw11'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*_launch.py')),
        (f'share/{package_name}/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hugo Costelha',
    maintainer_email='hugo.costelha@ipleiria.pt',
    description='tw11 - Behavior Trees',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tree-data-gathering = tw11.01_data_gathering:main',
            'battery-check = tw11.02_battery_check:main',
            'simple-task = tw11.03_simple_task:main',
            'simple-task-nav2 = tw11.03_simple_task_nav2:main',
        ],
    },
)
