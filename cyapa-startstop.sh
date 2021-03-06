#!/bin/sh
#
# $FreeBSD$
#

# PROVIDE: cyapa
# REQUIRE: dbus hald
#

. /etc/rc.subr

name="cyapa"
rcvar="cyapa_enable"
load_rc_config $name

start_cmd="cyapa_start"
stop_cmd="cyapa_stop"
load_cmd="cyapa_loadmod"
extra_commands="load"

halcmddir="/usr/local/bin/"

check_and_load()
{
	local name=$1
	kldstat -n $name > /dev/null 2>&1 || kldload $name
}

cyapa_loadmod()
{
	# TODO: I2C bus driver should be configurable
	check_and_load ig4
	check_and_load cyapa
	service moused restart
}

cyapa_start()
{
	cyapa_loadmod
	sleep 1
	if [ -x ${halcmddir}/hal-device ]; then
		ls /dev/cyapa* | while read devfile; do
			halbasename=`basename "$devfile" | sed -e 's/-/_/g'`
			udi="/org/freedesktop/Hal/devices/$halbasename"
			n=0
			ok=0
			while [ $ok -eq 0 ];do
				${halcmddir}/hal-device -a $udi << __END__
freebsd.device_file='$devfile'
freebsd.driver='smb'
freebsd.unit=$n
info.parent = '/org/freedesktop/Hal/devices/computer'
info.capabilities = {'input', 'input.mouse'}
info.category = 'input'
info.product = 'Cypress APA I2C Trackpad'
info.udi = '$udi'
input.device = '/dev/sysmouse'
input.x11_driver = 'mouse'
__END__
				if [ $? = 0 ];then
					ok=1
				else
					sleep 3
				fi
			done
			n=`expr $n + 1`
		done
	fi
}

cyapa_stop()
{
	if [ -x ${halcmddir}/hal-device ]; then
		${halcmddir}/lshal | \
		  grep "^udi = '/org/freedesktop/Hal/devices/cyapa" | \
		  cut -d= -f2 | sed -e "s/^ *'//; s/'$//" | while read udi; do
		  ${halcmddir}/hal-device -r $udi
		done
	fi
}

run_rc_command "$1"
