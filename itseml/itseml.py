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
import netaddr

from shutil import copyfile
from string import Template

TMPL_PATH = os.path.join(os.path.dirname(__file__), "templates")
CONF_PATH = "/var/run/itseml/"
IP_NETWORK= "10.1.1.0/24"
SUBNET_PLEN = 30

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

# create formatter
formatter = logging.Formatter('%(asctime)s :: %(levelname)s :: %(message)s')

# log to stderr
stream_handler = logging.StreamHandler()
stream_handler.setLevel(logging.DEBUG)
stream_handler.setFormatter(formatter)

logger.addHandler(stream_handler)

def start_env(params, envname):
    directory = "/var/run/itseml/%s/mw/config" % (envname)
    logging.info("Creating destination directory: %s", directory)

    try:
        distutils.dir_util.mkpath(directory)
    except DistutilsFileError as e:
        logging.error("Failed to create directory: %s" % (e))

    generate_configuration(params, envname)

    _service_action('start', envname)
    _network_conf(int(params['id']))
    logging.info("Started environment: %s" % (envname))
    return "200 OK"

def _mac_to_gn_addr(hw_addr):
    gn_addr = '0000' + hw_addr.replace(':', '')
    return ':'.join([gn_addr[i:i+4] for i in range(0, len(gn_addr), 4)])

def _get_if_mac(iface):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    info = fcntl.ioctl(s.fileno(), 0x8927,  struct.pack('256s', iface[:15]))
    return ':'.join(['%02x' % ord(char) for char in info[18:24]])

def _network_conf(envnum):
    network = netaddr.IPNetwork(IP_NETWORK)
    subnets = list(network.subnet(SUBNET_PLEN))
    net = subnets[envnum]

    local = str(netaddr.IPAddress(net.first+1))
    remt = str(netaddr.IPAddress(net.first+2))

    cmds = []
    cmds.append("ip netns exec env%d ip a a %s/%d dev eth1" % (envnum, remt, SUBNET_PLEN))
    cmds.append("ip netns exec env%d ip r a default via %s" % (envnum, local))
    cmds.append("ip a a %s/%d dev ctl%d" % (local, SUBNET_PLEN, envnum))
    logging.info("Applying IP configuration")
    for cmd in cmds:
        out = subprocess.check_call(cmd, shell=True)

def _service_action(action, envname):
    svcs = ['eml-netns', 'eml-itsnet', 'eml-mwtun', 'eml-mw-server']

    services = ' '.join([x + '@%s' % (envname) for x in svcs])

    subprocess.check_call("systemctl %s %s" % (action, services), shell=True)

def stop_env(envname, envid):
    _service_action('stop', envname)
    logging.info("Stopped environment: %s" % (envname))
    return "200 OK"

def status_env(envname):
    _service_action('is-active -q', envname)

def generate_configuration(params, envname):
    # Copy files that don't need modifications
    filelist = ['log4j.properties', 'config/spatconfig.xml', 'config/trafficlight.xml',
            'config/v2xconfig.xml', 'config/vehiclediagnosticconfig.xml', 'config/ldmservice.xml',
            'config/rhsservice.xml', 'config/dsrcmitigation.xml']

    src = [os.path.join(TMPL_PATH, "mw", x) for x in filelist]
    dst = [os.path.join("/var/run/itseml/%s/mw" % (envname), x) for x in filelist]
    for i,_ in enumerate(src):
        copyfile(src[i], dst[i])

    # Generate configuration files from JSON
    _fields = {
        'gn': 'itsnet.conf',
        'denm': 'mw/config/denservice.xml',
        'position': 'mw/config/positionproviderconfig.xml',
        'cam': 'mw/config/caconfig.xml',
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
        "station_type": 15,
    }
    logging.info("Creating configuration for mw-server: mw/choirconf.xml")
    _process(tpl_params, "mw/choirconf.xml")

    # Generate trafficlight.xml
    num_tl = len(params.get('trafficlight').get('states'))
    states = params.get('trafficlight').get('states')
    durations = params.get('trafficlight').get('durations')

    tl_conf_list = []
    for tl, (state, duration) in enumerate(zip(states, durations)):
        tmpl = """\t\t<traffic%d length="%d" """ % (tl, len(state))
        tl_line = ""
        for i, (s, d) in enumerate(zip(state, duration)):
           tl_line += """t{0}="{2}" s{0}="{1}" """.format(i, s, d)

        #tmpl += ' '.join(tl_line)
        tl_conf_list.append(tmpl + tl_line + "/>")


    tl_params = {
        "num_tl": num_tl,
        "tl_conf": '\n'.join(tl_conf_list),
    }
    _process(tl_params, "mw/config/trafficlight.xml")
    logging.info("Creating configuration for mw-server: mw/config/trafficlight.xml")

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
        "stationid": rng.randint(1,65535),
        "gn": {
            "gn_addr": "0:0:0:1",
            "geobc_fwd_alg": 0,
        },
        "denm": {
            "forwarding": True,
            "forceactionid": False,
            "autoupdate": False,
        },
    }


    defaults = _dict_update(defaults, params)

    envname = "env" + str(defaults["id"])

    logging.debug("Processing received JSON:\n%s", pprint.pformat(defaults))

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
