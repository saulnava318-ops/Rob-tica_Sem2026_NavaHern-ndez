from setuptools import find_packages, setup

package_name = 'p0_py'

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
    maintainer='arrgusr',
    maintainer_email='rivascf@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'primer_nodo = p0_py.primer_nodo:main',
            'subscriber_nodo = p0_py.subscriber_nodo:main',
            'scara_test_01 = p0_py.scara_control:main',
            'q_plan = p0_py.q_plan:main',
            'ws_plan = p0_py.ws_plan:main'
        ],
    },
)