from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'final_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssafy',
    maintainer_email='ssafy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ptp_move_panel = final_project.ptp_move_panel:main',
            'main_prog = final_project.main_prog:main',
            'fixed_main_prog = final_project.fixed_main_prog:main',
            'test_main_prog = final_project.test_main_prog:main',

            'obj_detect = final_project.obj_detect:main',
            'adv_main_prog = final_project.adv_main_prog:main',
            'ptp_move_board = final_project.ptp_move_board:main',
            'ptp_move_back = final_project.ptp_move_back:main',
        ],
    },
)
