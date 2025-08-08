from setuptools import setup, find_packages

setup(
    name='panda_base',
    version='0.0.0',
    packages=find_packages('src'),      # 假设你的源码都在 src/ 下
    package_dir={'': 'src'},
    entry_points={
        'console_scripts': [
            'panda_control = panda_base.scripts.panda_control:main',
        ],
    },
)
