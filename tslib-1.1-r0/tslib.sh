#!/bin/sh

if [ -e /dev/ttymxc2 ]; then
    TSLIB_TSDEVICE=/dev/ttymxc0

    export TSLIB_TSDEVICE
fi

if [ -e /dev/fb0 ]; then
    TSLIB_FBDEVICE=/dev/fb0

    export TSLIB_FBDEVICE
fi

if [ -e /etc/ts.conf ]; then
        TSLIB_CONFFILE=/etc/ts.conf
        export TSLIB_CONFFILE
fi

if [ -e /usr/lib/ts ]; then
        TSLIB_PLUGINDIR=/usr/lib/ts
        export TSLIB_PLUGINDIR
fi

TSLIB_CALIBFILE=/etc/pointercal

