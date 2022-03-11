from setuptools import setup

package_name = 'tw02'

setup(
    name=package_name,
    version='3.0.0',
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
    description='tw02 - Basic Navigation',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation = tw02.navigation:main'
        ],
    },
)
