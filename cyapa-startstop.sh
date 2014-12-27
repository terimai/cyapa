#!/bin/sh
case $1 in
start)
	ls /dev/cyapa* | while read devfile; do
		halbasename=`basename "$devfile" | sed -e 's/-/_/g'`
		udi="/org/freedesktop/Hal/devices/$halbasename"
		n=0
		hal-device -a $udi << __END__
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
	;;
stop)
	lshal | grep "^udi = '/org/freedesktop/Hal/devices/cyapa" | \
	 cut -d= -f2 | sed -e "s/^ *'//; s/'$//" | while read udi; do
		hal-device -r $udi
	done
	;;
*)
	echo "usage $0 {start|stop}"
esac
