#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import distutils.dir_util
import fcntl
import logging
import json
import os
import socket
import struct
import subprocess
import sys
import pprint
import netaddr

from shutil import copyfile
from string import Template

TMPL_PATH = "templates/"
CONF_PATH = "/var/run/itseml/"

# Port used for iptables redirection
BASE_PORT = 8080

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
    _network_conf(params['id'], "add")
    logging.info("Started environment: %s" % (envname))

def _mac_to_gn_addr(hw_addr):
    gn_addr = '0000' + hw_addr.replace(':', '')
    return ':'.join([gn_addr[i:i+4] for i in range(0, len(gn_addr), 4)])

def _get_if_mac(iface):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    info = fcntl.ioctl(s.fileno(), 0x8927,  struct.pack('256s', iface[:15]))
    return ':'.join(['%02x' % ord(char) for char in info[18:24]])

def _network_conf(envnum, action):
    action = "a" if action == "add" else "d"
    network = netaddr.IPNetwork("10.1.1.0/24")
    subnets = list(network.subnet(30))
    net = subnets[envnum-1]

    local = str(netaddr.IPAddress(net.first+1))
    remt = str(netaddr.IPAddress(net.last-1))
    port = BASE_PORT + envnum - 1

    cmds = []
    cmds.append("ip netns exec env%d ip a %s %s/30 dev eth1" % (envnum, action, remt))
    cmds.append("ip netns exec env%d ip r %s default via %s" % (envnum, action, local))
    if action == "d":
        cmds.reverse()
    cmds.append("ip a %s %s/30 dev ctl%d" % (action, local, envnum))
    logging.info("Applying IP configuration")
    for cmd in cmds:
        out = subprocess.check_call(cmd, shell=True)

    _iptables_rules(remt, port, action.upper())


def _iptables_rules(addr, port, action):

    cmds = []
    cmds.append("iptables -t nat -%s PREROUTING -p tcp --dport %d -j DNAT --to-destination %s:8080" % (action, port, addr))
    cmds.append("iptables -%s FORWARD -p tcp --dport %d -m state --state NEW,ESTABLISHED,RELATED -j ACCEPT" % (action, port))

    logging.info("Applying iptables rules")
    for cmd in cmds:
        out = subprocess.check_call(cmd, shell=True)


def _service_action(action, envname):
    svcs = ['eml-netns', 'eml-itsnet', 'eml-mwtun', 'eml-mw-server',
		'eml-gpsfwd', 'eml-gpsd', 'eml-gpspipe']
    if "is-active" in action:
        svcs.remove('eml-gpspipe')
    for svc in svcs:
        subprocess.check_call("systemctl %s %s@%s" % (action, svc, envname), shell=True)

def stop_env(envname, envid):
    _network_conf(envid, "del")
    _service_action('stop', envname)
    logging.info("Stopped environment: %s" % (envname))

def status_env(envname):
    _service_action('is-active -q', envname)

def generate_configuration(params, envname):
    # Copy files that don't need modifications
    filelist = ['log4j.properties', 'config/spatconfig.xml', 'config/trafficlight.xml',
            'config/v2xconfig.xml', 'config/vehiclediagnosticconfig.xml', 'config/ldmservice.xml',
            'config/rhsservice.xml']

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
        "station_id": params.get("id"),
        "station_type": 5,
    }
    logging.info("Creating configuration for mw-server: choirconf.xml")
    _process(tpl_params, "mw/choirconf.xml")


def process_message(params):
    envname = "env" + str(params["id"])

    logging.debug("Processing received JSON:\n%s", pprint.pformat(params))

    try:
        status_env(envname)
    except subprocess.CalledProcessError, e:
        if params['action'] == 'start':
            start_env(params, envname)
        elif params['action'] == 'stop':
            logging.info("Environment %s already stopped", envname)
    else:
        if params['action'] == 'start':
            logging.info("Environment %s already started", envname)
        elif params['action'] == 'stop':
            stop_env(envname, params["id"])

if __name__ == '__main__':
    msg = json.load(sys.stdin)
    process_message(msg)
