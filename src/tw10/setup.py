from setuptools import setup
from glob import glob

package_name = 'tw10'

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
    description='tw10 - Actions',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_task = tw10.simple_task:main',
            'simple_task_nav2 = tw10.simple_task_nav2:main',
            'action_move2pos = tw10.ActionMove2Pos:main',
            'action_play_sound = tw10.ActionPlaySound:main',
            'action_rotate2angle = tw10.ActionRotate2Angle:main',
            'action_recharge = tw10.ActionRecharge:main',
            'action_stop = tw10.ActionStop:main'
        ],
    },
)
