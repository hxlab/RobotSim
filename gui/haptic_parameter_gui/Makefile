.PHONY: install run run-ros check

PYTHON ?= python3

install:
	$(PYTHON) -m pip install -e .

run:
	$(PYTHON) -m haptic_parameter_gui.main --transport console

run-ros:
	$(PYTHON) -m haptic_parameter_gui.main --transport ros2

check:
	PYTHONPYCACHEPREFIX=/tmp/haptic_parameter_gui_pycache $(PYTHON) -m compileall haptic_parameter_gui setup.py
