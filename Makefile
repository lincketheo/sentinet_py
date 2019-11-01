
PY=/usr/bin/python3.7

.PHONY: all
all: build


.PHONY: install
install:
	sudo ${PY} setup.py install

.PHONY: build
build: 
	${PY} setup.py build
