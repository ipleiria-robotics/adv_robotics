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
            'teleop = lw2.teleop:main',
            'task_bt = lw2.task_bt:main',
            'task_fsm = lw2.task_fsm:main',
            'task_fsm_yasmin = lw2.task_fsm_yasmin:main',
            'ekf_icp_localization = lw2.ekf_icp_localization:main',
            'path_navigation = lw2.path_navigation:main',
            'action_move2pos = lw2.ActionMove2Pos:main',
            'action_move2pose = lw2.ActionMove2Pose:main',
            'action_move_forklift = lw2.ActionMoveForklift:main',
            'action_move_visual_servoing = lw2.ActionMoveVisualServoing:main',
            'action_speak_text = lw2.ActionSpeakText:main',
            'action_rotate2angle = lw2.ActionRotate2Angle:main',
            'action_recharge = lw2.ActionRecharge:main',
            'action_stop = lw2.ActionStop:main'
        ],
    },
)
