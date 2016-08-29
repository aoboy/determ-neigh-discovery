CONTIKI = $(DROPBOX_HOME)/neighbor-discovery/nddev-contiki

ifndef TARGET
TARGET=sky
endif

#...Deterministic Neighbor Discovery with Epidemics...
#CONTIKI_SOURCEFILES += dnde-neigh-G.c generic-dnde-G.c

#..Group Merge using multi channel..
CONTIKI_SOURCEFILES += dnde-neigh-G.c MJ-G-Merge.c

DEFINES=NETSTACK_MAC=nullmac_driver,NETSTACK_RDC=generic_driver,CC2420_CONF_AUTOACK=0
DEFINES+=CONF_CHANNEL_SIZE=1,CONF_NETWORK_SIZE=105
DEFINES+=HOPCOUNT_FILTER_NDISC=2,CONF_ND_DUTYCYCLE=5

DEFINES+=IN_INDRIYA=1
#DEFINES+=IN_TWIST=1
DEFINES+=CONF_ASYMMETRIC=1


install-generic:
#	make clear
	make example-generic-dnde-G.upload

	#cp SL-TlogT-example.sky ./binaries/SL-TlogT-ASYM15example.exe

clear:
	rm -rf *.sky *.exe symbols.* obj_* *~
	make clean

motelist:
	MOTES=$(shell $(MOTELIST) 2>&- | grep USB | \
	cut -f 4 -d \  | \
	perl -ne 'print $$1 . " " if(m-(/dev/\w+)-);')

reload:
	make sky-reset


include $(CONTIKI)/Makefile.include

