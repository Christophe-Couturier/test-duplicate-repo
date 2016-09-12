#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import collections
import distutils.dir_util
import fcntl
import logging
import os
import os.path
import socket
import struct
import subprocess
import sys
import pprint
import random
import json

from shutil import copyfile
from string import Template

TMPL_PATH = os.path.join(os.path.dirname(__file__), "templates")
CONF_PATH = "/var/run/itseml/"

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# create formatter
formatter = logging.Formatter('%(asctime)s :: %(levelname)s :: %(message)s')

# log to stderr
stream_handler = logging.StreamHandler()
stream_handler.setLevel(logging.DEBUG)
stream_handler.setFormatter(formatter)

logger.addHandler(stream_handler)

def start_env(params, envname):
    generate_configuration(params, envname)

    _service_action('start', envname)
    if 'intersections' in params.get('map'):
        map_sender(params.get('map'), envname)
        mqtt_notify(envname)
    logging.info("Started environment: %s" % (envname))
    return "200 OK"

def _mac_to_gn_addr(hw_addr):
    gn_addr = '0000' + hw_addr.replace(':', '')
    return ':'.join([gn_addr[i:i+4] for i in range(0, len(gn_addr), 4)])

def _get_if_mac(iface):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    info = fcntl.ioctl(s.fileno(), 0x8927,  struct.pack('256s', iface[:15]))
    return ':'.join(['%02x' % ord(char) for char in info[18:24]])

def _service_action(action, envname):
    svcs = ['eml-netns', 'eml-ip-setup', 'eml-itsnet', 'eml-mwtun', 'eml-mw-server']

    services = ' '.join([x + '@%s' % (envname) for x in svcs])

    logging.info("{0} services: {1}".format(action, services))
    subprocess.check_call("systemctl %s %s" % (action, services), shell=True)

def stop_env(envname, envid):
    logging.info("Stopping eml-mapsender@%s service" % (envname))
    out = subprocess.check_call("systemctl stop eml-mapsender@%s" % (envname), shell=True)
    logging.info("Stopping eml-mqtt-notify@%s service" % (envname))
    out = subprocess.check_call("systemctl stop eml-mqtt-notify@%s" % (envname), shell=True)
    _service_action('stop', envname)
    logging.info("Stopped environment: %s" % (envname))
    return "200 OK"

def status_env(envname):
    _service_action('is-active -q', envname)

def generate_configuration(params, envname):
    # Copy files that don't need modifications
    filelist = ['log4j.properties', 'config/spatconfig.xml',
            'config/v2xconfig.xml', 'config/ldmservice.xml']

    src = [os.path.join(TMPL_PATH, "mw", x) for x in filelist]
    dst = [os.path.join("/var/run/itseml/%s/mw" % (envname), x) for x in filelist]
    for i,_ in enumerate(src):
        copyfile(src[i], dst[i])

    # Generate configuration files from JSON
    _fields = {
        'gn': 'itsnet.conf',
        'denm': 'mw/config/denservice.xml',
        'position': 'mw/config/positionproviderconfig.xml',
        'ivi': 'mw/config/iviservice.xml',
    }

    params['gn']['lat'] =  params['position']['lat']
    params['gn']['lon'] =  params['position']['lon']

    def _process(field, filename):
        # Replace True with "true" and False with "false"
        for k, v in field.iteritems():
            if type(field[k]) == bool:
                field[k] = str(v).lower()

        with open(os.path.join(TMPL_PATH, filename), 'r') as f, \
            open(os.path.join(CONF_PATH, envname, filename), 'w') as dst:
            tpl = Template(f.read())
            dst.write(tpl.substitute(field))

    for k, v in _fields.iteritems():
        if k in params:
            logging.info("Creating configuration for %s: %s", k, v)
            _process(params[k], v)

    # Generate choirconf.xml
    tpl_params = {
        "station_id": params.get("stationid"),
        "station_type": params.get("station_type"),
    }
    logging.info("Creating configuration for mw-server: mw/choirconf.xml")
    _process(tpl_params, "mw/choirconf.xml")

    # Generate trafficlight.xml
    num_tl = len(params.get('trafficlight').get('states'))
    states = params.get('trafficlight').get('states')
    durations = params.get('trafficlight').get('durations')

    tl_conf_list = []
    for tl, (state, duration) in enumerate(zip(states, durations)):
        tl_line = ""
        st_dur_list = [(x,y) for x,y in zip(state, duration) if y != 0 ]
        for i, (s, d) in enumerate(st_dur_list):
            tl_line += """t{0}="{2}" s{0}="{1}" """.format(i, s, d)

        tmpl = """\t\t<traffic%d length="%d" """ % (tl, len(st_dur_list))
        tl_conf_list.append(tmpl + tl_line + "/>")


    tl_params = {
        "num_tl": num_tl,
        "interval": params.get('trafficlight').get('interval', 500),
        "tl_conf": '\n'.join(tl_conf_list),
    }

    if tl_params['interval'] == 0:
        tl_params['interval'] = 500

    _process(tl_params, "mw/config/trafficlight.xml")
    logging.info("Creating configuration for mw-server: mw/config/trafficlight.xml")

def mqtt_notify(envname):
    logging.info("Starting eml-mqtt-notify@%s service" % (envname))
    out = subprocess.check_call("systemctl start eml-mqtt-notify@%s" % (envname), shell=True)

def map_sender(params, envname):
    mapjson = json.dumps({'map':params})
    with open(os.path.join(CONF_PATH, envname, 'map.json'), 'w') as f:
        f.write(mapjson)
    logging.info("Starting eml-mapsender@%s service" % (envname))
    out = subprocess.check_call("systemctl start eml-mapsender@%s" % (envname), shell=True)

def stop_all():
    _service_action("stop", "*");

def _dict_update(source, overrides):
    """Update a nested dictionary

    Modify ``source`` in place.
    """
    for key, value in overrides.iteritems():
        if isinstance(value, collections.Mapping) and value:
            returned = _dict_update(source.get(key, {}), value)
            source[key] = returned
        else:
            source[key] = overrides[key]
    return source

def process_message(params):
    rng = random.SystemRandom()
    defaults = {
        "stationid": rng.randint(1, 65535),
        "station_type": 15,
        "gn": {
            "gn_addr": "0:0:0:1",
            "geobc_fwd_alg": 0,
        },
        "denm": {
            "forwarding": True,
            "forceactionid": False,
            "autoupdate": False,
        },
        "ivi": {
            "autoupdate": False,
        },
        "trafficlight": {
            "states": [],
            "durations": [],
        },
        "map": {
            "msgID": 18,
            "msgSubID": 1,
            "msgIssueRevision": 1,
        },
    }


    defaults = _dict_update(defaults, params)

    logging.debug("Processing received JSON:\n%s", pprint.pformat(defaults))

    if defaults['action'] == 'stop_all':
        stop_all()
        return "200 OK"

    envname = "env" + str(defaults["id"])

    directory = "/var/run/itseml/%s/mw/config" % (envname)
    logging.info("Creating destination directory: %s", directory)

    try:
        distutils.dir_util.mkpath(directory)
    except DistutilsFileError as e:
        logging.error("Failed to create directory: %s" % (e))

    if defaults['action'] == 'start':
        defaultsjson = json.dumps(defaults)
        with open(os.path.join(CONF_PATH, envname, 'request.json'), 'w') as f:
            f.write(defaultsjson)

    response = "304 Not modified"

    try:
        status_env(envname)
    except subprocess.CalledProcessError, e:
        if defaults['action'] == 'start':
            response = start_env(defaults, envname)
        elif defaults['action'] == 'stop':
            logging.info("Environment %s already stopped", envname)
    else:
        if defaults['action'] == 'start':
            logging.info("Environment %s already started", envname)
        elif defaults['action'] == 'stop':
            response = stop_env(envname, defaults["id"])
    return response
