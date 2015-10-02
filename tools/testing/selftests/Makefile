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

all:
	for TARGET in $(TARGETS); do \
		make -C $$TARGET; \
	done;

run_tests: all
	for TARGET in $(TARGETS); do \
		make -C $$TARGET run_tests; \
	done;

run_pstore_crash:
	make -C pstore run_crash

clean:
	for TARGET in $(TARGETS); do \
		make -C $$TARGET clean; \
	done;
