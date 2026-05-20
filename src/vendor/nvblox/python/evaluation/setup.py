from setuptools import setup, find_packages

with open('requirements.txt', 'r', encoding='utf-8') as f:
    install_requires = [line.strip() for line in f.readlines() if line]

with open('README.md', 'r', encoding='utf-8') as f:
    long_description = f.read()

setup(name='nvblox_evaluation',
      version='0.0.0',
      description='Scripts for evaluating nvblox.',
      author='nvblox team.',
      author_email='amillane/dtingdahl/vramasamy/remos@nvidia.com',
      long_description=long_description,
      long_description_content_type='text/markdown',
      install_requires=install_requires,
      include_package_data=True,
      packages=find_packages())
