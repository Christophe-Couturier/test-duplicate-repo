#!/bin/sh

CURR_VER="$(python setup.py --version)"
CURR_IDX=1
LAST_VERSTR=$(cat .version)
LAST_IDX=$(echo $LAST_VERSTR | cut -d- -f2)
LAST_VER=$(echo $LAST_VERSTR | cut -d- -f1)

if [ $LAST_VER = $CURR_VER ]; then
	CURR_IDX=$(($LAST_IDX + 1))
fi

echo $CURR_VER-$CURR_IDX > .version
echo $CURR_VER-$CURR_IDX
