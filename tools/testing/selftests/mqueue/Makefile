all:
	gcc -O2 -lrt mq_open_tests.c -o mq_open_tests
	gcc -O2 -lrt -lpthread -lpopt -o mq_perf_tests mq_perf_tests.c

include ../lib.mk

override define RUN_TESTS
	@./mq_open_tests /test1 || echo "selftests: mq_open_tests [FAIL]"
	@./mq_perf_tests || echo "selftests: mq_perf_tests [FAIL]"
endef

clean:
	rm -f mq_open_tests mq_perf_tests
