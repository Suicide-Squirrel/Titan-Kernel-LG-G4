all:

run_tests:
	@./on-off-test.sh || echo "cpu-hotplug selftests: [FAIL]"

run_full_test:
	@/bin/bash ./on-off-test.sh -a || echo "cpu-hotplug selftests: [FAIL]"

clean:
