from freecad_dist import fc_install                 # custom freecad install
from setuptools.core import setup
import freecad_frame

setup(cmdclass={'install': fc_install},
      install_requires=[],
      name='freecad_frame',
      version=freecad_frame.version,
      description='some functionality for frame-building',
      url='my_website',
      author='looooo',
      license='LGPL2',
      packages=["freecad_beam", "icons"],
      package_data = {"": ["*.svg"]})     # not std files (.py)