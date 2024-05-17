from setuptools import setup
from glob import glob

package_name = 'tw09'

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
    description='tw09 - EKF-based Localization with ICP',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_icp_localization = tw09.ekf_icp_localization:main',
            'path_navigation = tw09.path_navigation:main',
            'publish_fixed_path = tw09.publish_fixed_path:main',
            'nav2_test.py = tw09.nav2_test.py'
        ],
    },
)
