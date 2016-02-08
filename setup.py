#!/usr/bin/env python2

from distutils.core import setup

setup(name='itseml',
      version='0.1',
      description='ITS emulator helper',
      author='Marios Makassikis',
      author_email='marios.makassikis@yogoko.fr',
      url='https://www.yogoko.fr',
      packages=['itseml'],
      package_data={'itseml': ['templates/*', 'templates/mw/*',
                               'templates/mw/config/*']},
     )
