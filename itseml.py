#!/usr/bin/env python2
import distutils.dir_util
import fcntl
import json
import os
import socket
import struct
import subprocess
import sys

from string import Template

ENV_PATH  = "/etc/itseml/envs/"
TMPL_PATH  = "/etc/itseml/templates/"
ENV_LIST  = []
CONF_PATH = "/var/lib/itseml/"

# Port used for iptables redirection
BASE_PORT = 8080

# Operations:
#  - list: List available environments
#  - start: Start an environment
#  - start-all: Start all environments
#  - stop: Stop an environment
#  - stop-all: Stop all environments
#  - enable: Delete any services created previously
#  - enable-all: Delete any services created previously
#  - disable: Delete any services created previously
#  - disable-all: Delete any services created previously
#  - status: Show status for a given environment


def start_env(params, envname):
    try:
        distutils.dir_util.mkpath("/var/run/itseml/%s/mw/config" % (envname))
    except DistutilsFileError as e:
        print ("Failed to create directory: %s" % (e))

    setup_itsnet(params, envname)

    _service_action('start', envname)
    _network_conf(params['id'], "add")
    print ("Started environnment: %s" % (envname))

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

    envname = "env" + str(envnum)

    cmds = []
    cmds.append("ip netns exec env%d ip a %s %s/30 dev eth1" % (envnum, action, remt))
    cmds.append("ip netns exec env%d ip r %s default via %s" % (envnum, action, local))
    if action == "d":
        cmds.reverse()
    cmds.append("ip a %s %s/30 dev ctl%d" % (action, local, envnum))
    action = action.upper()

    cmds.append("iptables -t nat -%s PREROUTING -p tcp --dport %d -j DNAT --to-destination %s:8080" % (action, port, remt))
    cmds.append("iptables -%s FORWARD -p tcp --dport %d -m state --state NEW,ESTABLISHED,RELATED -j ACCEPT" % (action, port))

    for cmd in cmds:
        out = subprocess.check_call(cmd, shell=True)


def _service_action(action, envname):
    for svc in ['eml-netns', 'eml-itsnet', 'eml-mwtun', 'eml-mw-server', 'eml-gpsfwd',
		'eml-gpsd', 'eml-gpspipe']:
        subprocess.check_call("systemctl %s %s@%s" % (action, svc, envname), shell=True)

def stop_env(envname):
    _network_conf(params['id'], "del")
    _service_action('stop', envname)
    print ("Stopped environnment: %s" % (envname))

def status_env(envname):
    _service_action('status', envname)

def setup_itsnet(params, envname):
    itsnet_params = {
        "SAPSocket": "/var/run/itseml/%s/GN_SAP.sock" % (envname),
        "geobc_fwd_alg": "%s" % (params.get("geobc_fwd_alg", 0)),
    }

    with open(os.path.join(TMPL_PATH, 'itsnet.conf.tpl'), 'r') as f, \
        open("/var/run/itseml/%s/itsnet.conf" % (envname), 'w') as dst:
        gn_tpl = Template(f.read())
        dst.write(gn_tpl.substitute(itsnet_params))

def setup_mwserver(params):
    pass

def setup_env(envname, params):
    setup_itsnet(params['itsnet'])
    setup_mwserver(params['mw-server'])

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Setup ITS emulation environnment')
    parser.add_argument('operation', help='Operation. Must be one of: start | stop');
    #parser.add_argument('-v', dest='verbose', action='store_true')
    ##parser.add_argument('start', dest='_start', action='store_true')
    parser.add_argument('env', metavar='N',
                   help='environment name')


if __name__ == '__main__':
    params = json.load(sys.stdin)

    envname = "env" + str(params["id"])

    if params['action'] == 'start':
        start_env(params, envname)
    elif params['action'] == 'stop':
        stop_env(envname)


