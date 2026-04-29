# =============================================================================
# RBPS — convenience wrapper around CMake.
#
# This Makefile is for development ergonomics only; the canonical build is
# CMake. Anything you can do here, you can also do with `cmake -S . -B build`
# directly.
#
# Quick start:
#   make             # show help
#   make build       # configure + build everything
#   make build test  # build, then run the test suite
#   make docs        # generate the Doxygen site
#   make clean       # wipe the build directory
# =============================================================================

# ── Tunables (override on the command line, e.g. `make BUILD_TYPE=Debug build`)
BUILD_DIR  ?= build
BUILD_TYPE ?= Release
JOBS       ?= $(shell nproc 2>/dev/null || echo 4)
WITH_VISR  ?= ON
WITH_DOCS  ?= OFF

CMAKE      ?= cmake
CTEST      ?= ctest

# Filter tests by name regex: `make test TEST=gjk` runs only matching tests.
TEST       ?=

CMAKE_FLAGS = -S . -B $(BUILD_DIR) \
              -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
              -DRBPS_BUILD_VISR=$(WITH_VISR) \
              -DRBPS_BUILD_DOCS=$(WITH_DOCS)

CACHE = $(BUILD_DIR)/CMakeCache.txt

# ── Default goal ─────────────────────────────────────────────────────────────
.DEFAULT_GOAL := help

.PHONY: help configure build test list-tests docs run demo \
        clean rebuild debug release reconfigure

# ── Targets ──────────────────────────────────────────────────────────────────

help:  ## Show this help (default)
	@printf "RBPS Makefile — convenience wrapper around CMake.\n\n"
	@printf "Targets:\n"
	@awk 'BEGIN{FS=":.*##"} /^[a-zA-Z][a-zA-Z0-9_-]*:.*##/ {printf "  \033[36m%-13s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@printf "\nVariables (override with VAR=value on the command line):\n"
	@printf "  BUILD_DIR  = %s\n" "$(BUILD_DIR)"
	@printf "  BUILD_TYPE = %s   (Release | Debug | RelWithDebInfo)\n" "$(BUILD_TYPE)"
	@printf "  JOBS       = %s\n" "$(JOBS)"
	@printf "  WITH_VISR  = %s   (ON | OFF — Raylib + ImGui visualizer)\n" "$(WITH_VISR)"
	@printf "  WITH_DOCS  = %s  (ON | OFF — Doxygen docs target)\n" "$(WITH_DOCS)"
	@printf "  TEST       = %s    (regex filter for \`make test\`)\n" "$(TEST)"
	@printf "\nExamples:\n"
	@printf "  make build              # configure + build\n"
	@printf "  make build test         # build, then run the test suite\n"
	@printf "  make test TEST=gjk      # run only tests whose name matches /gjk/\n"
	@printf "  make BUILD_TYPE=Debug build\n"
	@printf "  make docs               # build + open hint for the Doxygen site\n"
	@printf "  make rebuild            # clean + build from scratch\n"

configure: $(CACHE)  ## Run CMake configure (auto-runs on first build)

reconfigure:  ## Force a fresh CMake configure (without wiping the cache)
	$(CMAKE) $(CMAKE_FLAGS)

$(CACHE):
	$(CMAKE) $(CMAKE_FLAGS)

build: $(CACHE)  ## Build all targets (auto-configures on first run)
	$(CMAKE) --build $(BUILD_DIR) -j$(JOBS)

test: build  ## Build, then run tests (TEST=<regex> to filter)
	@if [ -n "$(TEST)" ]; then \
	    echo "Running tests matching /$(TEST)/"; \
	    $(CTEST) --test-dir $(BUILD_DIR) -R "$(TEST)" --output-on-failure -j$(JOBS); \
	else \
	    $(CTEST) --test-dir $(BUILD_DIR) --output-on-failure -j$(JOBS); \
	fi

list-tests: $(CACHE)  ## List every registered ctest target
	@$(CTEST) --test-dir $(BUILD_DIR) -N

docs:  ## Generate the Doxygen documentation site
	$(CMAKE) $(CMAKE_FLAGS) -DRBPS_BUILD_DOCS=ON
	$(CMAKE) --build $(BUILD_DIR) --target docs -j$(JOBS)
	@printf "\nOpen: file://%s/docs/html/index.html\n" "$(abspath $(BUILD_DIR))"

run: build  ## Build, then run the visr_demo binary
	@if [ ! -x "$(BUILD_DIR)/visr_demo" ]; then \
	    echo "visr_demo is not built (WITH_VISR=$(WITH_VISR)). Try: make run WITH_VISR=ON"; \
	    exit 1; \
	fi
	@$(BUILD_DIR)/visr_demo

demo: run  ## Alias for `run`

debug:  ## Configure + build in Debug mode
	@$(MAKE) --no-print-directory BUILD_TYPE=Debug build

release:  ## Configure + build in Release mode
	@$(MAKE) --no-print-directory BUILD_TYPE=Release build

rebuild: clean build  ## Wipe the build directory and rebuild from scratch

clean:  ## Remove the build directory
	@rm -rf $(BUILD_DIR)
	@echo "Removed $(BUILD_DIR)/"
