from setuptools import setup

package_name = 'sample_rclpy'

def reg_script(script_name):
    return f'{script_name} = {package_name}.{script_name}:main'

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
    maintainer='shun',
    maintainer_email='tamushun.0818@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            reg_script('count_action_server'),
            reg_script('count_action_client_async')
        ],
    },
)
