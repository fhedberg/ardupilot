include /usr/share/cdbs/1/rules/debhelper.mk

BOARDS = sitl pxf navio raspilot bbbmini minlure erlebrain2 bhat qflight
FRAMES = quad tri hexa y6 octa octa-quad heli

define ardupilot_template
	mkdir -p debian/${1}-${2}/etc/ardupilot
	mkdir -p debian/${1}-${2}/etc/defaults
	mkdir -p debian/${1}-${2}/usr/bin
	mkdir -p debian/${1}-${2}/lib/systemd/system
	mkdir -p debian/${1}-${2}/etc/avahi/services
	mkdir -p debian/${1}-${2}/var/lib/ardupilot/logs

	cp ../Tools/linux/avahi/ardupilot.service debian/${1}-${2}/etc/avahi/services/${1}.service
	cp ../Tools/linux/systemd/${1}.service debian/${1}-${2}/lib/systemd/system/${1}
	cp ../Tools/linux/default/${1} debian/${1}-${2}/etc/defaults/${1}
endef

define arducopter_template
	$(call ardupilot_template,arducopter,${1})

	$(foreach frame,${FRAMES},
		make clean ${1}-${frame}
		cp ArduCopter.elf debian/arducopter-${1}/usr/bin/arducopter-${frame}
	)
	
	mkdir -p debian/arducopter-${1}/etc/ardupilot/defaults/copter
	cp ../Tools/Frame_params/*.param debian/arducopter-${1}/etc/ardupilot/defaults/copter
endef

define arduplane_template
	$(call ardupilot_template,arduplane,${1})

	make clean ${1}
	cp ArduPlane.elf debian/arduplane-${1}/usr/bin/arduplane
endef
