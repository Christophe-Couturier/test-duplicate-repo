#!/bin/sh

DESTDIR="$1"

UNIT_DIR=/lib/systemd/system
DOC_DIR=/usr/share/doc/itseml
TMPL_DIR=/var/lib/itseml

# Unit templates
find . -type f -iname '*.service' | while read f; do
	install -o root -g root -m 644 -D ${f} \
				${DESTDIR}/${UNIT_DIR}/$f
done

# Create directories
mkdir -p ${DESTDIR}/${DOC_DIR}
mkdir -p ${DESTDIR}/${TMPL_DIR}

# Helper scripts
install -o root -g root -m 755 -D setup_netns \
				${DESTDIR}/usr/bin/setup_netns
install -o root -g root -m 755 -D gen_itsnet_conf \
				${DESTDIR}/usr/bin/gen_itsnet_conf
# Main script
python setup.py install --prefix=${DESTDIR}/usr

# Convenient symlink when we want to feed data from stdin
ln -s ${DESTDIR}/usr/lib/python2.7/site-packages/itseml/itseml.py ${DESTDIR}/usr/bin/itseml

sed -i ${DESTDIR}/usr/bin/itseml -e "s,^TMPL_PATH = .*,TMPL_PATH = \"${TMPL_DIR}/templates\","

# WebSocket frontend
install -o root -g root -m 755 -D ws.py \
				${DESTDIR}/usr/bin/ws-itseml

# Templates
find templates -type f | while read f; do
	install -o root -g root -m 644 -D ${f} \
				${DESTDIR}/${TMPL_DIR}/$f
done

# Docs
install -o root -g root -m 644 -D README \
				${DESTDIR}/${DOC_DIR}/README
install -o root -g root -m 644 -D sample.nginx.conf \
				${DESTDIR}/${DOC_DIR}/sample.nginx.conf
