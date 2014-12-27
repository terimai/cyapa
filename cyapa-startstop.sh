#!/bin/sh
#
# $FreeBSD$
#

# PROVIDE: cyapa
# REQUIRE: hald
#

. /etc/rc.subr

name="cyapa"
rcvar="cyapa_enable"
load_rc_config $name

start_cmd="cyapa_start"
stop_cmd="cyapa_stop"
load_cmd="cyapa_loadmod"
extra_commands="load"

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
	ls /dev/cyapa* | while read devfile; do
		halbasename=`basename "$devfile" | sed -e 's/-/_/g'`
		udi="/org/freedesktop/Hal/devices/$halbasename"
		n=0
		/usr/local/bin/hal-device -a $udi << __END__
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
		n=`expr $n + 1`
	done
}

cyapa_stop()
{
	/usr/local/bin/lshal | grep "^udi = '/org/freedesktop/Hal/devices/cyapa" | \
	 cut -d= -f2 | sed -e "s/^ *'//; s/'$//" | while read udi; do
		/usr/local/bin/hal-device -r $udi
	done
}

run_rc_command "$1"
