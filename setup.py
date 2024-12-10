from setuptools import find_packages, setup

package_name = 'examen_final_robotica_2024'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hinfinity',
    maintainer_email='dacalvimontes@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_camera = examen_final_robotica_2024.read_camera:main',
            'examen_3 = examen_final_robotica_2024.examen_3:main',
            'principal = examen_final_robotica_2024.principal:main',
        ],
    },
)
