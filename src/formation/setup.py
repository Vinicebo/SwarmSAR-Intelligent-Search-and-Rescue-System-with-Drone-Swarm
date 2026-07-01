from setuptools import setup

package_name = 'formation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vinícius da Luz',
    maintainer_email='viniciusdaluzlcp@gmail.com',
    description='Boids-based swarm formation and spacing control for SwarmSAR.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
