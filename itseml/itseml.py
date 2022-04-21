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
import time
import re

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
    time.sleep(5)
    push_MIB_configuration(params, envname)
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
    # Copy log4j.properties, yogoko-middleware.license & choirconf.xml
    filelist = [
        'log4j.properties',
        'yogoko-middleware.license',
        'choirconf.xml'
    ]

    src = [os.path.join(TMPL_PATH, "mw", x) for x in filelist]
    dst = [os.path.join("/var/run/itseml/%s/mw" % (envname), x) for x in filelist]
    for i,_ in enumerate(src):
        copyfile(src[i], dst[i])

    # Generate configuration file from JSON
    _fields = {
        'itsnet': 'itsnet.conf'
    }

    def _process(field, filename):
        # Replace True with "true" and False with "false"
        for k, v in field.iteritems():
            if type(field[k]) == bool:
                field[k] = str(v).lower()

        with open(os.path.join(TMPL_PATH, filename), 'r') as f, \
            open(os.path.join(CONF_PATH, envname, filename), 'w') as dst:
            tpl = Template(f.read())
            dst.write(tpl.substitute(field))

    # Generate configuration files
    for k, v in _fields.iteritems():
        logging.info("Creating configuration for %s: %s", k, v)
        _process(params['its_station'][k], v)

def push_MIB_configuration(params, envname):
    #Push MIB configuration from JSON
    env_num = int(envname[3:])
    _, rem_ip = _addr_pair(env_num)
    for service, conf_values in sorted(params['its_station']['services'].iteritems(), key=lambda t:t[0]):
        logging.info("Pushing MIB configurations for %s service" % (service))
        cmd = "mwconfig mib set -t %s " % (service[3:])
        for key, value in sorted(conf_values.iteritems(), key=lambda t:t[0]):
            formatted_value = value
            if type(value) == bool:
                formatted_value = str(value).lower()
            elif type(value) == str:
                formatted_value = '\"' + value + '\"'
            elif type(value) == unicode:
                formatted_value = '\"' + str(value) + '\"'
            cmd += "'%s=%s' " % (key, formatted_value)
    	cmd += "-d tcp://%s:49154" % (rem_ip)
    	out = subprocess.check_call(shlex.split(cmd))
    	out = subprocess.check_call(shlex.split("mwconfig control start -t %s -d tcp://%s:49154" % (service[3:], rem_ip)))

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
    out = re.sub(r'\(id: \d+\)', '', out)
    out = out.replace('env', ' ').replace('\n', '').strip()

    s = {}
    envs = out.split(' ')

    for i in envs:
        if len(i) > 0:
            _, rem_ip = _addr_pair(int(i))
            appid = rng.randint(2**16, 2**30)
            res = subprocess.check_output(shlex.split("mwcache -a %d -c %d -l -d tcp://%s:49154/" % (appid, cacheid, rem_ip)))
            logging.info("Here is the return of cache list : %s" % (res))
            if not res:
                s[i] = 0
            else:
                d = json.loads(res)
                s[i] = len(d)

    return s

def get_denm():
    return get_cache_list(11)

def get_ivi():
    return get_cache_list(14)

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
    out = re.sub(r'\(id: \d+\)', '', out)
    out = out.replace('env', ' ').replace('\n', '').strip()

    s = {}
    envs = out.split(' ')

    for i in envs:
        if len(i) > 0:
            path = "/var/run/itseml/env%s/request.json" % (i)
            with open(path, 'r') as f:
                config = json.load(f)
                s[i] = 0
                if "40-spatservice" in config['its_station']['services']:
                    if "index.size" in config['its_station']['services']['40-spatservice']:
                        if config['its_station']['services']['40-spatservice']['index.size'] > 0 : s[i] = 1

    return s

def get_cam():
    out = subprocess.check_output(["ip", "netns"])
    out = re.sub(r'\(id: \d+\)', '', out)
    out = out.replace('env', ' ').replace('\n', '').strip()

    s = {}
    envs = out.split(' ')

    for i in envs:
        if len(i) > 0:
            s[i] = 0
            path = "/var/run/itseml/env%s/request.json" % (i)
            with open(path, 'r') as f:
                config = json.load(f)
                if "40-CaService" in config['its_station']['services']:
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
              #"map_stats": get_map(),
              "ivi_stats": get_ivi(),
              #"capture_enabled": get_capture(),
    }
    yield json.dumps(stats)


def process_message(params):
    defaults = {
        "its_station": {
            "itsnet": {
                "gn_addr": "0:0:0:1",
                "geobc_fwd_alg": 0,
            },
            "services": {
                "10-IdentityManager": {
#                    "service.start": True,
                    "choir.applicationid": 1,
                    "choir.identitycacheid": 1,
                    "identity.stationid": rng.randint(1, 65535),
                    "identity.stationtype": 5,
                    "identity.idchangeenabled": False,
                    "security.enabled": False,
                    "security.signmessages": False,
                },
                "10-Pvt": {
#                    "service.start": True,
                    "choir.applicationid": 20,
                    "choir.vdpcacheid": 0,
                    "pvtconfig.recomputeinterval": 2000,
                    "sources.count": 7,
                    "source1.profile": "time",
                    "source1.name": "ntpprovider",
                    "source1.enabled": True,
                    "source1.feedmsgid": 207,
                    "source1.timeout": 1800000,
                    "source2.profile": "native",
                    "source2.name": "recorder",
                    "source2.enabled": False,
                    "source2.feedmsgid" :204,
                    "source2.timeout": 5000,
                    "source3.profile": "autotalks",
                    "source3.name": "autotalksprovider",
                    "source3.enabled": False,
                    "source3.feedmsgid": 201,
                    "source3.timeout": -1,
                    "source3.filtermode": 2,
                    "source3.fixspeedresetheadingthreshold": 1.0,
                    "source3.fixspeedupdateheadingthreshold": 2.0,
                    "source4.profile": "native",
                    "source4.name": "staticprovider",
                    "source4.enabled": True,
                    "source4.feedmsgid": 206,
                    "source4.timeout": -1,
                    "source4.fields": "ls2004",
                    "source5.profile": "gpsd",
                    "source5.name":"gpsprovider",
                    "source5.enabled": False,
                    "source5.feedmsgid": 205,
                    "source5.timeout": 2000,
                    "source6.profile": "user",
                    "source6.name": "userdata",
                    "source6.enabled": True,
                    "source6.feedmsgid": 203,
                    "source6.timeout": -1,
                    "source6.filtermode": 0,
                    "source7.profile": "native",
                    "source7.name": "staticprovider",
                    "source7.enabled": True,
                    "source7.timeout": -1
                },
                "10-V2XService": {
#                    "service.start": True,
                    "ipv6.enabled": True,
                    "mwtun1.channel": "cch",
                    "mwtun1.iface": "tun0",
                    "mwtun1.group": "ff02::1234",
                    "mwtun1.header": True
                },
                "20-NtpProvider": {
                    "choir.messageid": 207,
                    "ntpserver.address": "0.pool.ntp.org",
                    "ntpserver.port": 123
                },
                "20-StaticProvider": {
                    "choir.messageid": 206,
                    "staticposition.latitude": 48.135,
                    "staticposition.longitude": -1.622,
                    "staticposition.altitude": 182,
                    "staticposition.speed": 1.4,
                    "staticposition.heading": 189.2,
                    "staticposition.climb": 0,
                    "staticposition.ls2004": 5
                },
                "40-CaService": {
                    "pubsub.senderid": 61,
                    "pubsub.outcam": 22,
                    "pubsub.incam": 32,
                    "pubsub.owncam": 42,
                    "cache.inCacheId": 1,
                    "cache.vdpcacheid": 0,
                    "cache.stationPropertiesId": 3,
                    "cache.pathhistorycacheid": 9,
                    "socket.cam" : "gn+btp://cch:2001",
                    "camparameters.protocolversion": 1,
                    "camparameters.messageid": 2,
                    "camparameters.vehiclelengthconfind": 0,
                    "recvcontrol.dropownmsgs": False,
                    "sendcontrol.generationcontrol": True,
                    "sendcontrol.genmaxinterval": 200,
                    "sendcontrol.genmininterval": 100,
                    "sendcontrol.genminintervaldcc": 100,
                    "sendcontrol.consecutivecam": 3,
                    "sendcontrol.lowfrequencycontrol": True,
                    "sendcontrol.genminintervallf": 500,
                    "sendcontrol.mitigationcontrol": True,
                    "sendcontrol.mitigationinterval": 1000,
                    "securityparameters.signmessages": False,
                    "securityparameters.securitycontrol": False,
                    "securityparameters.aid": 36,
                    "lifetime.value": 40,
                    "protectedzone.index": 0,
                    "zone0.type": 0,
                    "zone0.latitude": 43.5512784,
                    "zone0.longitude": 10.3002593,
                    "zone0.zoneid": 1,
                    "zone0.radius": 100,
                    "zone0.expiration": 417381883
                },
                "50-LdmService": {
                    "pubsub.camreceiverid": 90,
                    "pubsub.incam": 32,
                    "pubsub.owncam": 42,
                    "pubsub.denmreceiverid": 94,
                    "pubsub.indenm": 45,
                    "pubsub.owndenm": 65,
                    "pubsub.spatreceiverid": 89,
                    "pubsub.inspat": 24,
                    "pubsub.ownspat": 44,
                    "pubsub.inmap": 23,
                    "pubsub.ownmap": 43,
                    "pubsub.ivireceiverid": 95,
                    "pubsub.inivi": 37,
                    "pubsub.ownivi": 67,
                    "cache.ldmcamcacheid": 10,
                    "cache.ldmeventcacheid": 11,
                    "cache.ldmtlcacheid": 12,
                    "cache.ldmprotectedcacheid": 13,
                    "cache.ldmsigncacheid": 14,
                    "cache.vdpcacheid": 0,
                    "station.enable": True,
                    "station.maxdistance": 1000,
                    "event.enable": True,
                    "event.maxdistance":2000,
                    "event.relevancedistance": 1000,
                    "event.relevancedistancefromtrace": 20,
                    "event.relevanceheading": 45,
                    "event.relevancedistancenoheading": 40,
                    "trafficlight.enable": True,
                    "trafficlight.maxdistance": 1000,
                    "trafficlight.relevancedistance": 200,
                    "trafficlight.lanerelevancedistance": 5,
                    "signage.enable": True,
                    "signage.maxdistance": 1000,
                    "signage.relevancedistance": 200,
                    "signage.relevancedistancefromtrace": 20,
                    "signage.relevanceanglefromtrace": 20.0,
                    "camrelevance.frequency": 100,
                    "camrelevance.timeout": 2000,
                    "denmrelevance.frequency": 500,
                    "spatmaprelevance.frequency": 100,
                    "spatmaprelevance.timeout": 2000,
                    "ivirelevance.frequency": 500
                }
            }
        }
    }

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
