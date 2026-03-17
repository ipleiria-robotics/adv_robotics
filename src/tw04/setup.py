from setuptools import setup
from glob import glob

package_name = 'tw04'

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
    description='tw04 - Basic Localization using Trilateration',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_navigation = tw04.path_navigation:main',
            'localization = tw04.localization:main',
            'publish_fixed_path = tw04.publish_fixed_path:main'
        ],
    },
)
