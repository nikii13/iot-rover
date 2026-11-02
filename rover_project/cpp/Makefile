# Compila el servicio de encoders. Sin dependencias externas: solo g++ y las
# cabeceras del kernel (paquete linux-libc-dev, ya presente en Raspberry Pi OS).
CXX      ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -pedantic

all: encodered

encodered: encoder.cpp quadrature.hpp
	$(CXX) $(CXXFLAGS) -o $@ encoder.cpp

test_quadrature: test_quadrature.cpp quadrature.hpp
	$(CXX) $(CXXFLAGS) -o $@ test_quadrature.cpp

test: test_quadrature
	./test_quadrature

clean:
	rm -f encodered test_quadrature

.PHONY: all test clean
