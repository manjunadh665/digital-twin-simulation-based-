from setuptools import setup

package_name = 'digital_twin'

setup(
    name=package_name,
    version='0.0.1',
    packages=['pythonfiles'],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'anomaly_detector = pythonfiles.anomaly_detection:main',
        ],
    },
)