import os
from glob import glob
from setuptools import setup

package_name = 'simulator'

# The repository keeps launch/, worlds/, models/, and config/ at the repo
# root (see the top-level project structure), two levels above this
# package's setup.py (repo_root/src/simulator/setup.py).
repo_root = os.path.join(os.path.dirname(__file__), '..', '..')

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join(repo_root, 'launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join(repo_root, 'worlds', '*.world'))),
        (os.path.join('share', package_name, 'models', 'quadrotor'),
            glob(os.path.join(repo_root, 'models', 'quadrotor', '*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join(repo_root, 'config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vinícius da Luz',
    maintainer_email='viniciusdaluzlcp@gmail.com',
    description='Drone spawner, world manager, and communication range filter for SwarmSAR.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
