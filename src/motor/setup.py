from setuptools import setup, glob

package_name = 'motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/param', glob.glob('param/*')),
    ],
    install_requires=['setuptools', 'gpiozero'],
    zip_safe=True,
    maintainer='grisw',
    maintainer_email='grisw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = motor.node:main'
        ],
    },
)
