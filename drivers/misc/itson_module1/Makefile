THIS_DIR=$(realpath $(dir $(lastword $(MAKEFILE_LIST))))

ANDROID_MAKE_FILE = Android.mk

ifeq ($(TARGET_BUILD_VARIANT),user)
$(warning Purging logs)
#EXTRA_CFLAGS += -g0 -s -DPURGE_LOG
EXTRA_LDFLAGS += --strip-debug
endif

ifeq ($(DO_PURGE_LOG),1)
EXTRA_CFLAGS += -DPURGE_LOG
$(warning Purging all logs)
endif

EXTRA_CFLAGS += -DKERNELVERSION=$(VERSION)
$(warning Kernel version is $(VERSION))

# UIDs that have "level 1" access (e.g. ability to MediaMap)
EXTRA_CFLAGS += -DITSON_SECURE_LEVEL1_UID="{ 1013, 1000 }"
# UIDs that have "level 2" access (i.e. the Itson service)
EXTRA_CFLAGS += -DITSON_SECURE_LEVEL2_UID="{ 1500 }"

CFLAGS_MODULE += -fno-pic

itson_module1-objs := ItsOnAPI.o platform_lock.o platform_timestamp.o		\
			platform_timer.o platform_config.o platform_alloc.o	\
			platform_procfile.o platform_print.o
obj-$(CONFIG_ITSON_API) += itson_module1.o


obfuscate:
	echo Creating obfuscate.h
	perl rename.pl --ignore itson_logic/itson_filter.o itson_logic/module2.ko > itson_logic/obfuscate.h

clean-files := *.ko *.o *.mod.c modules.order Android.mk init.itson.rc .*.cmd

clean:
	rm -f *.ko *.o *.mod.c modules.order Android.mk init.itson.rc .*.cmd

