from setuptools import find_packages, setup

package_name = 'uva_asv_arov_control'

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
    maintainer='malori',
    maintainer_email='abo7fg@virginia.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'formal_methods_experiment = uva_asv_arov_control.formal_methods_experiment:main',
            'vu_z_control = uva_asv_arov_control.vu_zcontrollerSimple:main'
        ],
    },
)
