from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'operator_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'templates'),
            glob('templates/*'),
        ),
        (
            os.path.join('share', package_name, 'static', 'css'),
            glob('static/css/*'),
        ),
        (
            os.path.join('share', package_name, 'static', 'js'),
            glob('static/js/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tjfife',
    maintainer_email='tjwillett@gmail.com',
    description='Web-based operator interface for the SAADS platform.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'operator_server = operator_interface.server:main',
        ],
    },
)
