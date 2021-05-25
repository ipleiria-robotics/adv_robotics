from setuptools import setup
from glob import glob

package_name = 'lw2'

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
        (f'share/{package_name}/sounds', glob('sounds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hugo Costelha',
    maintainer_email='hugo.costelha@ipleiria.pt',
    description='lw2 - 2nd Assignment',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task = lw2.task:main',
            'ekf_icp_localization = lw2.ekf_icp_localization:main',
            'path_navigation = lw2.path_navigation:main',
            'action_move2pos = lw2.ActionMove2Pos:main',
            'action_play_sound = lw2.ActionPlaySound:main',
            'action_rotate2angle = lw2.ActionRotate2Angle:main',
            'action_recharge = lw2.ActionRecharge:main',
            'action_stop = lw2.ActionStop:main'
        ],
    },
)
