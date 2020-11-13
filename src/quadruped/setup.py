from setuptools import setup

package_name = 'quadruped'

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
    maintainer=[
        'ashutoshshrm529',
        'ekanshgupta92',
        'Damn-It-Tech',
        'archit2802',
        'bhavika-g'
    ],
    maintainer_email=[
        'ashutoshshrm529@gmail.com',
        'ekanshgupta92@gmail.com',
        'mehuljain.jaipur@gmail.com',
        'archit.mudugu2802@gmail.com',
        'bhavika.gopalani@gmail.com'
    ],
    description='This repository contains all the code for the autonomous quadruped being developed by the team.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # add later
        ],
    },
)
