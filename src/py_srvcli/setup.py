from setuptools import setup

package_name = 'py_srvcli'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='John-Henry Lim',
    maintainer_email='42513874+Interpause@users.noreply.github.com',
    description='Why do they not make this interactive',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = py_srvcli.add_service:main',
            'client = py_srvcli.add_client:main'
        ],
    },
)
