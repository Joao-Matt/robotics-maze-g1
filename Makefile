PYTHON_VERSION ?= 3.11.15
PYENV_ROOT ?= $(HOME)/.pyenv
PYENV ?= $(PYENV_ROOT)/bin/pyenv
PYTHON ?= $(PYENV_ROOT)/versions/$(PYTHON_VERSION)/bin/python
VENV ?= .venv
OPENSSL_ROOT ?= $(HOME)/.local/openssl
SEED ?= 1
DURATION ?= 3
VIEW_DURATION ?= 30
CONFIG ?= configs/default.yaml

.PHONY: setup smoke run view test clean

setup:
	@test -x "$(PYTHON)" || (echo "Python $(PYTHON_VERSION) was not found at $(PYTHON). Install it with: $(PYENV) install $(PYTHON_VERSION)" && exit 1)
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(PYTHON)" -m venv "$(VENV)"
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" -m pip install --upgrade pip
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" -m pip install -r requirements.txt

smoke:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/smoke_test.py

run:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/run_episode.py --seed "$(SEED)" --duration "$(DURATION)" --config "$(CONFIG)"

view:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" scripts/run_episode.py --seed "$(SEED)" --duration "$(VIEW_DURATION)" --config "$(CONFIG)" --viewer

test:
	@test -x "$(VENV)/bin/python" || (echo "Missing $(VENV). Run: make setup" && exit 1)
	LD_LIBRARY_PATH="$(OPENSSL_ROOT)/lib:$$LD_LIBRARY_PATH" "$(VENV)/bin/python" -m pytest tests

clean:
	rm -rf .pytest_cache
