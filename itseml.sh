#!/bin/bash -x


#MW_SERVER=/opt/mw-server/middleware-server.jar
MW_SERVER=/opt/middleware-server.jar
POS_SVC_ID=200
DAEMON_PIDS=
BRIDGE=v2xbr

cleanup() {
	for pid in $DAEMON_PIDS; do
		kill ${pid}
	done;
	stop_netns ${ENVNAME}
}

netns_exec() {
	ip netns exec ${ENVNAME} $@
}

start_netns() {
	NETNS="$1"
	ID=${NETNS##env}

	ip netns add ${NETNS}
	ip netns exec ${NETNS} ip a a 127.0.0.1/8 dev lo
	ip netns exec ${NETNS} ip a a ::1/128 dev lo
	ip netns exec ${NETNS} ip l s lo up

	ip link add vtx${ID} type veth peer name eth0 netns ${NETNS}
	ip link add ctl${ID} type veth peer name eth1 netns ${NETNS}

	ip link set vtx${ID} up master ${BRIDGE}
	ip link set ctl${ID} up

	if [ -n ${MAC_ADDR} ]; then
		ip netns exec ${NETNS} ip link set dev eth0 address ${MAC_ADDR}
	fi

	if [ -n ${CTL_IP} ]; then
		ip a a ${CTL_IP} dev ctl${ID}
	fi
	if [ -n ${ENV_IP} ]; then
		ip netns exec ${NETNS} ip a a ${ENV_IP} dev eth1
	fi

	ip netns exec ${NETNS} ip link set eth0 up
	ip netns exec ${NETNS} ip link set eth1 up
}

stop_netns() {
	NETNS="$1"
	ID=${NETNS##env}

	ip link del vtx${ID}
	ip link del ctl${ID}
	ip netns del ${NETNS}
}

start_itsnet() {
	mkdir -p /var/run/itseml/${ENVNAME}
	mkdir -p /var/lib/itseml/${ENVNAME}
	
	# Generate configuration
	ADDR=$(netns_exec ip -o l show eth0 | \
       		sed -e 's/^.*ether //' -e 's/://g' -e 's/^/0000/' | \
       		awk '{print $1;}' | sed -e 's/.\{4\}/&:/g' -e 's/:$//')

	# This file has been installed by cargeo6 package
	cp /usr/share/doc/cargeo6/itsnet.conf /var/lib/itseml/${ENVNAME}/itsnet.conf
	sed -i -e "s,^#IpcSocket.*,IpcSocket = /var/run/itseml/${ENVNAME}/GN_IPC.sock," \
       		-e "s,^#SAPSocket.*,SAPSocket = /var/run/itseml/${ENVNAME}/GN_SAP.sock," \
		-e "s,^#ItsnetLog.*,ItsnetLog = /var/lib/itseml/${ENVNAME}/itsnet.log," \
		-e "s/0:0:0:1/$ADDR/" \
		/var/lib/itseml/${ENVNAME}/itsnet.conf

	# Start itsnet and mwtun
	netns_exec itsnet -c /var/lib/itseml/${ENVNAME}/itsnet.conf -d > /dev/null &
	DAEMON_PIDS="$! ${DAEMON_PIDS}"
	
	netns_exec mwtun -i tun0 -s fe80::1234 -d ff02::1234 -c ff02::4567 -S /var/run/itseml/${ENVNAME}/GN_SAP.sock 1>/var/lib/itseml/${ENVNAME}/mwtun.log &
	DAEMON_PIDS="$! ${DAEMON_PIDS}"
}

start_mw_server() {
	cd ${MW_PATH}
	netns_exec java -jar ${MW_SERVER} 1>/var/lib/itseml/${ENVNAME}/mw-server.log &
	DAEMON_PIDS="$! ${DAEMON_PIDS}"
}

start_gpsd() {
	#netns_exec socat EXEC:"netcat_choir -a 2345 -m ${POS_SVC_ID} -s" TCP-LISTEN:4590 &
	ip netns exec ${ENVNAME} socat EXEC:"netcat_choir -a $RANDOM -m 200 -s" tcp4-listen:4590 &
	DAEMON_PIDS="$! ${DAEMON_PIDS}"
	netns_exec gpsd -N gpsd://localhost:4590 &
	DAEMON_PIDS="$! ${DAEMON_PIDS}"
}

configure_env() {
	start_netns ${ENVNAME}
	start_itsnet ${ENVNAME}
	start_mw_server ${ENVNAME}
	# gpsd get its' data from the middleware, so it needs to be started after
	sleep 5
	start_gpsd ${ENVNAME}
	echo $DAEMON_PIDS
	wait
}

trap cleanup QUIT TERM EXIT

. "$1"
configure_env ${ENVNAME}

#case "$1" in
#list)
#	ls "$ENV_PATH"
#;;
#start)
#	shift
#	ENVNAME="$1"
#	configure_env ${ENVNAME}
#;;
#start-all)
#	find ${ENV_PATH} -type f | while read ev; do start_env $(basename $ev); done
#;;
#stop)
#	shift
#	ENVNAME="$1"
#	stop_env ${ENVNAME}
#;;
#stop-all)
#	find ${ENV_PATH} -type f | while read ev; do stop_env $(basename $ev); done
#;;
#enable)
#	shift
#	ENVNAME="$1"
#	configure_env ${ENVNAME}
#	enable_env ${ENVNAME}
#;;
#enable-all)
#	find ${ENV_PATH} -type f | while read ev; do enable_env $(basename $ev); done
#;;
#disable)
#	shift
#	ENVNAME="$1"
#	start_env ${ENVNAME}
#;;
#disable-all)
#	find ${ENV_PATH} -type f | while read ev; do disable_env $(basename $ev); done
#;;
#*)
#	echo "Unrecognized operation"
#	exit 1
#esac
