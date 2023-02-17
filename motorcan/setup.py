from setuptools import setup

setup(name='motorcan',
      version='0.0.0',
      description='wrapper module for using tmotor and odrive motors over can',
      license='MIT',
      py_modules=['motorcan'],
      include_package_data=True,
      packages=["candatabase"],
      zip_safe=False
      )