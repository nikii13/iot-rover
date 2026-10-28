CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O2
LDLIBS ?= -latomic -llgpio

.DEFAULT_GOAL := all

all: encoder encoderd

encoder: encoder.hpp encoder.cpp standalone.cpp
	$(CXX) $(CXXFLAGS) encoder.cpp standalone.cpp -o $@ $(LDLIBS)

encoderd: encoder.hpp encoder.cpp encoderd.cpp
	$(CXX) $(CXXFLAGS) encoder.cpp encoderd.cpp -o $@ $(LDLIBS)

.PHONY: clean
clean:
	rm -f encoder encoderd

.PHONY: install
install: encoderd encoderd.service
	-systemctl disable --now encoderd.service >/dev/null 2>&1
	install -m 0755 -o root -g root encoderd /usr/local/bin/encoderd
	install -m 0644 -o root -g root encoderd.service /etc/systemd/system/encoderd.service
	systemctl daemon-reload
	systemctl enable --now encoderd.service

.PHONY: uninstall
uninstall:
	-systemctl disable --now encoderd.service >/dev/null 2>&1
	rm -f /usr/local/bin/encoderd
	rm -f /etc/systemd/system/encoderd.service
	rm -f /tmp/encoder.sock
	systemctl daemon-reload
