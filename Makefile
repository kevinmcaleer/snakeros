# SnakeROS development tasks.
#
# The whole stack runs on the MicroPython Unix port, so everything here works
# on a laptop with no hardware attached.

MICROPYTHON ?= micropython
COMPOSE     ?= docker compose -f docker/docker-compose.yml
ROS_DISTRO  ?= jazzy

.PHONY: help test unit integration cdr-diff rig-up rig-down rig-logs bench mpy package clean

help:
	@echo "make unit         - fast unit tests, no Docker needed"
	@echo "make rig-up       - start the micro-ROS Agent and a ROS 2 container"
	@echo "make integration  - end-to-end pub/sub against the real Agent"
	@echo "make cdr-diff     - diff our CDR against rclpy, both directions"
	@echo "make test         - unit + cdr-diff + integration"
	@echo "make bench        - throughput and memory benchmarks"
	@echo "make mpy          - cross-compile to .mpy (smaller, lower import peak)"
	@echo "make package      - regenerate the mip manifests"
	@echo "make rig-down     - tear the rig down"

unit:
	$(MICROPYTHON) tests/test_unit.py

rig-up:
	$(COMPOSE) up -d
	@echo "waiting for the Agent..."
	@sleep 5
	@docker ps --format '{{.Names}}  {{.Status}}' | grep snakeros

rig-down:
	$(COMPOSE) down -v

rig-logs:
	docker logs snakeros_agent --tail 50

# Ground truth from rclpy, then diff in both directions.
cdr-diff:
	docker exec snakeros_ros bash -lc 'rm -rf /tmp/w && mkdir -p /tmp/w'
	docker cp tests snakeros_ros:/tmp/w/tests
	docker exec snakeros_ros bash -lc 'source /opt/ros/$(ROS_DISTRO)/setup.bash && python3 /tmp/w/tests/rclpy_dump.py > /tmp/rclpy.json'
	docker cp snakeros_ros:/tmp/rclpy.json /tmp/rclpy.json
	$(MICROPYTHON) tests/test_cdr_vs_rclpy.py /tmp/rclpy.json
	docker cp /tmp/snakeros.json snakeros_ros:/tmp/snakeros.json
	docker exec snakeros_ros bash -lc 'source /opt/ros/$(ROS_DISTRO)/setup.bash && python3 /tmp/w/tests/rclpy_check.py /tmp/snakeros.json'

integration:
	./tests/integration.sh

test: unit cdr-diff integration
	@echo "all test suites passed"

bench:
	$(MICROPYTHON) tests/bench.py

# Cross-compile to .mpy. Cuts the flash footprint to ~38% of source and,
# more usefully on a tight board, ~32 KB off the peak heap during import.
mpy:
	@python3 -c "import mpy_cross" 2>/dev/null || { \
	  echo "mpy-cross not installed:  pip install mpy-cross"; exit 1; }
	python3 tools/build_package.py --mpy
	@echo ""
	@echo "Copy to a board with:"
	@echo "  mpremote fs rm -r :lib/snakeros      # remove the .py install first"
	@echo "  mpremote fs cp -r build/mpy/snakeros :lib/"

# Regenerate package.json and packages/*.json without cross-compiling.
package:
	python3 tools/build_package.py

clean:
	find . -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null || true
	find . -name '*.mpy' -delete
	rm -rf build dist
