from setuptools import setup
from glob import glob

package_name = 'tw06'

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
    description='tw06 - Search and planning',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'search_and_planning = tw06.search_and_planning:main',
        ],
    },
)
