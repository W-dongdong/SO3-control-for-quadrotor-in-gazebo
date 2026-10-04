from setuptools import setup

package_name = 'px4_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dong',
    maintainer_email='dong@example.com',
    description='PX4 SITL 入门飞控节点',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 名称 = 模块路径:函数名
            # 没有这一段，ros2 run px4_control offboard_control 会报 No executable found
            'offboard_control = px4_control.offboard_control:main',
        ],
    },
)
