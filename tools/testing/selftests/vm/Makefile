# Makefile for vm selftests

CC = $(CROSS_COMPILE)gcc
CFLAGS = -Wall

all: hugepage-mmap hugepage-shm  map_hugetlb thuge-gen
%: %.c
	$(CC) $(CFLAGS) -o $@ $^

TEST_PROGS := run_vmtests

include ../lib.mk

clean:
	$(RM) hugepage-mmap hugepage-shm  map_hugetlb
