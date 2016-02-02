#!/usr/bin/env python2
import argparse
import os
import errno
import distutils.dir_util
import glob
import subprocess
import struct
import socket
import fcntl
import yaml
import sys

from string import Template

ENV_PATH  = "/etc/itseml/envs/"
TMPL_PATH  = "/etc/itseml/templates/"
ENV_LIST  = []
CONF_PATH = "/var/lib/itseml/"

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

def list_envs():
    for env in ENV_LIST:
        print('\t%s' % env)

def configure_itsnet(params):

def start_env(envname):
    with open(os.path.join(ENV_PATH, envname +'.yml'), 'r') as f:
        conf = yaml.safe_load(f)

    setup_env(envname, params)

    try:
        mac = conf["itsnet"]["mac_addr"]
    except (KeyError):
        print ("Missing MAC addr parameter")


    _service_action('start', envname)
    print ("Started environnment: %s" % (envname))


def _mac_to_gn_addr(hw_addr):
    gn_addr = '0000' + hw_addr.replace(':', '')
    return ':'.join([gn_addr[i:i+4] for i in range(0, len(gn_addr), 4)])

def _get_if_mac(iface):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    info = fcntl.ioctl(s.fileno(), 0x8927,  struct.pack('256s', iface[:15]))
    return ':'.join(['%02x' % ord(char) for char in info[18:24]])


def _service_action(action, envname):
    for svc in ['eml-netns', 'eml-itsnet', 'eml-mwtun', 'eml-mw-server', 'eml-gpsfwd',
		'eml-gpsd', 'eml-gpspipe']:
        subprocess.check_call("systemctl %s %s@%s" % (action, svc, envname), shell=True)

def stop_env(envname):
    _service_action('stop', envname)
    print ("Stopped environnment: %s" % (envname))

def enable_env(envname):
    _service_action('enable', envname)

def disable_env(envname):
    _service_action('disable', envname)

def status_env(envname):
    _service_action('status', envname)

def start_all_envs():
    for env in ENV_LIST:
        start_env(env)

def stop_all_envs():
    for envname in ENV_LIST:
        _service_action('stop', envname)

def enable_all_env():
    for envname in ENV_LIST:
        _service_action('enable', envname)

def disable_all_envs():
    for envname in ENV_LIST:
        _service_action('disable', envname)

_ops = {
    'list'        : list_envs,
    'start'       : start_env,
    'start-all'   : start_all_envs,
    'stop'        : stop_env,
    'stop-all'    : stop_all_envs,
    'enable'      : enable_env,
    'enable-all'  : enable_all_env,
    'disable'     : disable_env,
    'disable-all' : disable_all_envs,
    'status'      : status_env,
}

def setup_itsnet(params):
    itsnet_params = {
        "gn_addr": "%s" % (_mac_to_gn_addr(mac)),
        "SAPSocket": "/var/run/itseml/%s/GN_SAP.sock" % (envname),
        "geobc_fwd_alg": "0",
    }

    try:
	distutils.dir_util.mkpath("/var/run/itseml/%s/" % (envname))
	#distutils.dir_util.mkpath("/var/lib/itseml/%s/" % (envname))
    except DistutilsFileError as e:
        print ("Failed to create directory: %s" % (e))

    with open(os.path.join(TMPL_PATH, 'itsnet.conf.tpl'), 'r') as f, \
        open("/var/lib/itseml/%s/itsnet.conf" % (envname), 'w') as dst:
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

    args = parser.parse_args()

    ENV_LIST = glob.glob(os.path.join(ENV_PATH, '*.yml'))
    ENV_LIST = [os.path.basename(x).replace('.yml', '') for x in ENV_LIST]

    if args.operation not in _ops:
        print ("Invalid operation requested")
    if args.env not in ENV_LIST:
        print ("Invalid environment name")
	

    _ops[args.operation](args.env)


