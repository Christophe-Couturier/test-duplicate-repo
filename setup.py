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
      scripts=['bin/gen_itsnet_conf', 'bin/setup_netns', 'bin/ws-itseml', 'bin/cli-itseml',
               'bin/eml-ip-setup', 'bin/publish_mqtt', 'bin/send_mapjson'],
      data_files=[('/usr/share/doc/itseml', ['README', 'doc/sample.nginx.conf',
                                             'doc/sample_json/start.json',
                                             'doc/sample_json/stop.json']),
                  ('/lib/systemd/system', ['service_files/eml-netns@.service',
                                           'service_files/eml-ip-setup@.service',
                                           'service_files/eml-itsnet@.service',
                                           'service_files/eml-mapsender@.service',
                                           'service_files/eml-mqtt-notify@.service',
                                           'service_files/eml-mw-server@.service',
                                           'service_files/eml-mwtun@.service',
                                           'service_files/eml-netns@.service',
                                           'service_files/ws-itseml.service'])]

     )
