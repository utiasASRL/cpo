from setuptools import setup

package_name = 'cpo_analysis'

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
    maintainer='ben',
    maintainer_email='ben.congram@robotics.utias.utoronto.ca',
    description='TODO: Package description',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['listener = cpo_analysis.plot_from_msg:main',
                            'plot_file = cpo_analysis.plot_from_file:main',
                            ],
    },
)
