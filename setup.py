#!/usr/bin/env python2

from distutils.core import setup
from datetime import datetime

setup(name='itseml',
      version='%s' % (datetime.utcnow().strftime('%Y.%m.%d')),
      description='ITS emulator helper',
      author='Marios Makassikis',
      author_email='marios.makassikis@yogoko.fr',
      url='https://www.yogoko.fr',
      packages=['itseml'],
      package_data={'itseml': ['templates/*', 'templates/mw/*',
                               'templates/mw/config/*']},
      scripts=['bin/gen_itsnet_conf', 'bin/setup_netns', 'bin/ws-itseml', 'bin/cli-itseml',
               'bin/eml-ip-setup', 'bin/start_cap', 'bin/stop_cap', 'bin/list_itss', 'bin/stop_itss'],
      data_files=[('/usr/share/doc/itseml', ['README', 'doc/sample.nginx.conf',
                                             'doc/sample_json/start.json',
                                             'doc/sample_json/stop.json']),
                  ('/lib/systemd/system', ['service_files/eml-netns@.service',
                                           'service_files/eml-ip-setup@.service',
                                           'service_files/eml-itsnet@.service',
                                           'service_files/eml-mw-server@.service',
                                           'service_files/eml-netns@.service',
                                           'service_files/ws-itseml.service',
                                           'service_files/license-mw-server.service']),
                  ('/etc/mw-license', ['mw-server/choirconf.xml',
                                       'mw-server/log4j.properties',
                                       'mw-server/yogoko-middleware.p12',
                                       'mw-server/truststore.jks'])]

     )
