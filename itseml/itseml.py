#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import collections
import distutils.dir_util
import fcntl
import logging
import netaddr
import os
import os.path
import shlex
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

IP_NETWORK= "10.1.1.0/24"
SUBNET_PLEN = 30
rng = random.SystemRandom()

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
    logging.info("Started environment: %s" % (envname))
    return """{"status": true}"""

def _mac_to_gn_addr(hw_addr):
    gn_addr = '0000' + hw_addr.replace(':', '')
    return ':'.join([gn_addr[i:i+4] for i in range(0, len(gn_addr), 4)])

def _addr_pair(envnum):
    """Return a tuple with local, remote IP addresses"""
    network = netaddr.IPNetwork(IP_NETWORK)
    subnets = list(network.subnet(SUBNET_PLEN))
    net = subnets[envnum]

    local = str(netaddr.IPAddress(net.first+1))
    remote = str(netaddr.IPAddress(net.first+2))

    return local, remote

def _get_if_mac(iface):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    info = fcntl.ioctl(s.fileno(), 0x8927,  struct.pack('256s', iface[:15]))
    return ':'.join(['%02x' % ord(char) for char in info[18:24]])

def _service_action(action, envname):
    svcs = ['eml-netns', 'eml-ip-setup', 'eml-itsnet', 'eml-mw-server']

    services = ' '.join([x + '@%s' % (envname) for x in svcs])

    logging.info("{0} services: {1}".format(action, services))
    subprocess.check_call(shlex.split("systemctl %s %s" % (action, services)))

def stop_env(envname, envid):
    _service_action('stop', envname)
    logging.info("Stopped environment: %s" % (envname))
    return """{"status": true}"""

def status_env(envname):
    _service_action('is-active -q', envname)

def generate_configuration(params, envname):
    # Copy files that don't need modifications
    filelist = ['log4j.properties', 'config/spatconfig.xml',
            'config/vdpconfig.xml',
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
        'cam': 'mw/config/caconfig.xml',
    }

    params['gn']['lat'] =  params['position']['lat']
    params['gn']['lon'] =  params['position']['lon']

    if params.get('cam').get('enable'):
        params['gn']['type'] = "gpsd"
    else:
        params['gn']['type'] = "static"


    def _process(field, filename):
        # Replace True with "true" and False with "false"
        for k, v in field.iteritems():
            if type(field[k]) == bool:
                field[k] = str(v).lower()

        with open(os.path.join(TMPL_PATH, filename), 'r') as f, \
            open(os.path.join(CONF_PATH, envname, filename), 'w') as dst:
            tpl = Template(f.read())
            dst.write(tpl.substitute(field))

    # Enable CA Protected Zone
    params['cam']['idx'] = len(params.get('cam').get('protectedzones'))

    if params['cam']['idx'] == 0 and params.get("station_type") == 15:
        selfzone = { "type": 0, "latitude": params['position']['lat'],
                 "longitude": params['position']['lon'] }
        params.get('cam').get('protectedzones').append(selfzone)
        params['cam']['idx'] = 1

    protected_zones_conf = []
    for idx, zone in enumerate(params.get('cam').get('protectedzones')):
        l = ["""%s="%s" """ % (k, v) for k,v in zone.items()]
        tmpl = """\t\t<zone%d %s/>""" % (idx, ''.join(l))
        protected_zones_conf.append(tmpl)

    params.get('cam')['protected_zones_conf'] = '\n'.join(protected_zones_conf)

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

    _fields['trafficlight'] = "mw/config/trafficlight.xml"
    params['trafficlight'] = tl_params

    # choirconf.xml parameters
    mw_params = {
        "station_id": params.get("stationid"),
        "station_type": params.get("station_type"),
    }

    _fields['mw'] = "mw/choirconf.xml"
    params['mw'] = mw_params

    # Generate configuration files
    for k, v in _fields.iteritems():
        if k in params:
            logging.info("Creating configuration for %s: %s", k, v)
            _process(params[k], v)

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

def get_envs():
    out = subprocess.check_output(["ip", "netns"])
    return out.count("env")

def get_cache_list(cacheid):
    out = subprocess.check_output(["ip", "netns"])
    out = out.replace('env', ' ').replace('\n', '').strip()

    s = {}
    envs = out.split(' ')

    for i in envs:
        if len(i) > 0:
            _, rem_ip = _addr_pair(int(i))
            appid = rng.randint(2**16, 2**30)
            res = subprocess.check_output(shlex.split("mwcache -a %d -c %d -o 0 -l -d tcp://%s:49154/" % (appid, cacheid, rem_ip)))
            d = json.loads(res)
            s[i] = len(d['objectList'])

    return s

def get_denm():
    return get_cache_list(2)

def get_ivi():
    return get_cache_list(3)

def get_map():
    out = subprocess.check_output(["ip", "netns"])
    out = out.replace('env', ' ').replace('\n', '').strip()

    s = {}
    envs = out.split(' ')

    for i in envs:
        if len(i) > 0:
            try:
                out = subprocess.check_call(shlex.split("systemctl is-active -q eml-mapsender@env%s" % (i)))
                s[i] = 1
            except subprocess.CalledProcessError, e:
                s[i] = 0

    return s


def get_spat():
    out = subprocess.check_output(["ip", "netns"])
    out = out.replace('env', ' ').replace('\n', '').strip()

    s = {}
    envs = out.split(' ')

    for i in envs:
        if len(i) > 0:
            path = "/var/run/itseml/env%s/mw/config/trafficlight.xml" % (i)
            with open(path, 'r') as f:
                for line in f:
                    if 'index size' in line:
                        res = line
            res = res.strip('\t\n></')
            res = res.replace('"', "").split('=')[1]
            if int(res) == 0:
                s[i] = 0
            else:
                s[i] = 1

    return s

def get_cam():
    out = subprocess.check_output(["ip", "netns"])
    out = out.replace('env', ' ').replace('\n', '').strip()

    s = {}
    envs = out.split(' ')

    for i in envs:
        if len(i) > 0:
            s[i] = 0
            path = "/var/run/itseml/env%s/mw/config/caconfig.xml" % (i)
            with open(path, 'r') as f:
                for line in f:
                    if 'service start="true"' in line:
                        s[i] = 1
    return s

def get_capture():
    svcs = ['itssnif', 'gn-itsnet', 'gn-mwtun', 'mw-server']

    services = ' '.join(svcs)

    logging.info("Checking if capture services are running: {0}".format(services))
    try:
        subprocess.check_call(shlex.split("systemctl is-active -q %s" % (services)))
    except (subprocess.CalledProcessError) as e:
        return False

    return True


def statistics():
    stats = { "num_stations": get_envs(),
              "cam_stats": get_cam(),
              "denm_stats": get_denm(),
              "spat_stats": get_spat(),
              "map_stats": get_map(),
              "ivi_stats": get_ivi(),
              "capture_enabled": get_capture(), }
    yield json.dumps(stats)


def process_message(params):
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
        "cam": {
            "enable": False,
            "idx": 0,
            "protectedzones": [],
        },
    }


    if params.get('cam'):
        params["cam"]["enable"] = True
    defaults = _dict_update(defaults, params)

    logging.debug("Processing received JSON:\n%s", pprint.pformat(defaults))

    if defaults['action'] == 'stop_all':
        stop_all()
        return """{"status": true}"""

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

    response = """{"status": true}"""

    try:
        status_env(envname)
    except subprocess.CalledProcessError, e:
        if defaults['action'] == 'start':
            try:
                response = start_env(defaults, envname)
            except KeyError, e:
                response = """ {"status": false, "message": "missing field %s" """ % (repr(e))
        elif defaults['action'] == 'stop':
            logging.info("Environment %s already stopped", envname)
    else:
        if defaults['action'] == 'start':
            logging.info("Environment %s already started", envname)
        elif defaults['action'] == 'stop':
            response = stop_env(envname, defaults["id"])
    return response
