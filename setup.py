from setuptools import find_packages, setup

package_name = 'bb8_navigate_to_goal'

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
    maintainer='Tzu-Tung Chen',
    maintainer_email='tchen604@gatech.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'test=bb8_navigate_to_goal.test:main',
        	'getObjectRange=bb8_navigate_to_goal.getObjectRange:main',
        	'goToGoal=bb8_navigate_to_goal.goToGoal:main',
        	'go=bb8_navigate_to_goal.go:main'
        ],
    },
)
