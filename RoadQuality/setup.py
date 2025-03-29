from setuptools import setup, find_packages

setup(
    name='RoadQuality',
    version='0.1.0',
    author='Your Name',
    author_email='your.email@example.com',
    description='A project to measure and analyze road quality using various sensors.',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/yourusername/RoadQuality',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=[
        'numpy',
        'matplotlib',
        'pynmea2',
        'pyserial',
        'fastestrplidar',
        'smbus2',
        'folium',
        'opencv-python',
    ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)