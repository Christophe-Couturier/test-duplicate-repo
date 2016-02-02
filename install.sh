#!/bin/sh

DESTDIR="$1"

# Unit templates
install -o root -g root -m 644 -D eml-netns@.service \
				${DESTDIR}/lib/systemd/system
install -o root -g root -m 644 -D eml-itsnet@.service \
				${DESTDIR}/lib/systemd/system
install -o root -g root -m 644 -D eml-mwtun@.service \
				${DESTDIR}/lib/systemd/system
install -o root -g root -m 644 -D eml-mw-server@.service \
				${DESTDIR}/lib/systemd/system
install -o root -g root -m 644 -D eml-gpsfwd@.service \
				${DESTDIR}/lib/systemd/system
install -o root -g root -m 644 -D eml-gpsd@.service \
				${DESTDIR}/lib/systemd/system
install -o root -g root -m 644 -D eml-gpspipe@.service \
				${DESTDIR}/lib/systemd/system

# Create directories
mkdir -p ${DESTDIR}/etc/itseml/templates

# Helper scripts
install -o root -g root -m 755 -D setup_netns \
				${DESTDIR}/usr/bin/setup_netns
install -o root -g root -m 755 -D itseml.py \
				${DESTDIR}/usr/bin/itseml
