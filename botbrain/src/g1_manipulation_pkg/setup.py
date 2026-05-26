from setuptools import find_packages, setup
from glob import glob

package_name = 'g1_manipulation_pkg'


def expand(patterns):
    files = []
    for p in patterns:
        files.extend(glob(p, recursive=True))
    return files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),

        # Launch
        (f'share/{package_name}/launch',
         expand(['launch/*.py'])),

        # Config
        (f'share/{package_name}/config',
         expand(['config/*.yaml'])),

        # URDF
        (f'share/{package_name}/description_files/urdf',
         expand(['description_files/urdf/*.urdf',
                 'description_files/urdf/*.xacro'])),

        # Meshes
        (f'share/{package_name}/description_files/meshes',
         expand(['description_files/meshes/**/*.STL'])),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BotBrain Dev',
    maintainer_email='dev@botbrain.io',
    description='G1 arm manipulation subsystem for BotBrain',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = g1_manipulation_pkg.manipulation.arm_controller:main',
            'dx3_controller = g1_manipulation_pkg.manipulation.dx3_hand:main',
            'interactive_marker = g1_manipulation_pkg.manipulation.interactive_marker:main',
            'arm_teleop_keyboard = g1_manipulation_pkg.scripts.arm_teleop_keyboard:main',
        ],
    },
)
