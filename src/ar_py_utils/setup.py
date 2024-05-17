from setuptools import setup

package_name = 'ar_py_utils'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hugo Costelha',
    maintainer_email='hugo.costelha@ipleiria.pt',
    description='Auxiliar files, functions and applications used in Advanced Robotics',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_truth_republisher = ar_py_utils.ground_truth_republisher:main',
        ],
    },
)
