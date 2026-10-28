from glob import glob
from setuptools import setup

setup(name='mrg_modimoop_test', version='0.1.0', packages=['mrg_modimoop_test'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/mrg_modimoop_test']),
                  ('share/mrg_modimoop_test', ['package.xml', 'README.md']),
                  ('share/mrg_modimoop_test/scenarios', glob('scenarios/*.yaml'))],
      tests_require=['pytest'],
      entry_points={'console_scripts': ['bench=mrg_modimoop_test.runner:main']})
