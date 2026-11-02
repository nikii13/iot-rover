# Atajos del proyecto. Ver README.md.
.PHONY: all build test test-cpp test-py start stop check clean

all: build

build:
	$(MAKE) -C cpp all

test: test-cpp test-py

test-cpp:
	$(MAKE) -C cpp test

test-py:
	python3 -m pytest -q tests

start:
	./scripts/start_services.sh

stop:
	./scripts/stop_services.sh

check:
	./scripts/check_system.sh

clean:
	$(MAKE) -C cpp clean
	rm -rf tests/__pycache__ lab/__pycache__ .pytest_cache
