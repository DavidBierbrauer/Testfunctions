from setuptools import setup

package_name = 'inception_test'

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
    maintainer='david',
    maintainer_email='david@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "node_one = inception_test.node_one:main",
            "node_two = inception_test.node_two:main",
            "node_three = inception_test.node_three:main",
            "node_four = inception_test.node_four:main"
        ],
    },
)
