TARGETS = breakpoints
TARGETS += cpu-hotplug
TARGETS += efivarfs
TARGETS += kcmp
TARGETS += memory-hotplug
TARGETS += mqueue
TARGETS += mount
TARGETS += net
TARGETS += pstore
TARGETS += ptrace
TARGETS += vm

TARGETS_HOTPLUG = cpu-hotplug
TARGETS_HOTPLUG += memory-hotplug

all:
	for TARGET in $(TARGETS); do \
		make -C $$TARGET; \
	done;

run_tests: all
	for TARGET in $(TARGETS); do \
		make -C $$TARGET run_tests; \
	done;

hotplug:
	for TARGET in $(TARGETS_HOTPLUG); do \
		make -C $$TARGET; \
	done;

run_hotplug: hotplug
	for TARGET in $(TARGETS_HOTPLUG); do \
		make -C $$TARGET run_full_test; \
	done;

clean_hotplug:
	for TARGET in $(TARGETS_HOTPLUG); do \
		make -C $$TARGET clean; \
	done;

run_pstore_crash:
	make -C pstore run_crash

INSTALL_PATH ?= install
INSTALL_PATH := $(abspath $(INSTALL_PATH))
ALL_SCRIPT := $(INSTALL_PATH)/run_kselftest.sh

install:
ifdef INSTALL_PATH
	@# Ask all targets to install their files
	mkdir -p $(INSTALL_PATH)
	for TARGET in $(TARGETS); do \
		mkdir -p $(INSTALL_PATH)/$$TARGET ; \
		make -C $$TARGET INSTALL_PATH=$(INSTALL_PATH)/$$TARGET install; \
	done;

	@# Ask all targets to emit their test scripts
	echo "#!/bin/bash" > $(ALL_SCRIPT)
	echo "cd \$$(dirname \$$0)" >> $(ALL_SCRIPT)
	echo "ROOT=\$$PWD" >> $(ALL_SCRIPT)

	for TARGET in $(TARGETS); do \
		echo "echo ; echo Running tests in $$TARGET" >> $(ALL_SCRIPT); \
		echo "echo ========================================" >> $(ALL_SCRIPT); \
		echo "cd $$TARGET" >> $(ALL_SCRIPT); \
		make -s --no-print-directory -C $$TARGET emit_tests >> $(ALL_SCRIPT); \
		echo "cd \$$ROOT" >> $(ALL_SCRIPT); \
	done;

	chmod u+x $(ALL_SCRIPT)
else
	$(error Error: set INSTALL_PATH to use install)
endif

clean:
	for TARGET in $(TARGETS); do \
		make -C $$TARGET clean; \
	done;

.PHONY: install
