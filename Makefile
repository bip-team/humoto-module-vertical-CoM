MAKE_FLAGS?=-j5

clean:
	rm -Rf build

all:
	mkdir -p build;
	cd build; cmake \
					-DHUMOTO_BUILD_TESTS=ON \
					-DHUMOTO_BUILD_REGRESSION_TESTS=ON \
					-DHUMOTO_MODULES_DIRS=`pwd`/../extra_modules/ \
					-DHUMOTO_BRIDGE_qpOASES=ON \
					-DHUMOTO_BRIDGE_config_yaml=ON \
					-DHUMOTO_MODULE_wpg04=ON \
					-DHUMOTO_MODULE_wpg05=ON \
					../humoto/
	cd build; ${MAKE} ${MAKE_FLAGS}

wpg04:
	mkdir -p build;
	cd build; cmake \
					-DHUMOTO_BUILD_TESTS=ON \
					-DHUMOTO_BUILD_REGRESSION_TESTS=ON \
					-DHUMOTO_MODULES_DIRS=`pwd`/../extra_modules/ \
					-DHUMOTO_BRIDGE_qpOASES=ON \
					-DHUMOTO_BRIDGE_obstacle_avoidance=ON \
					-DHUMOTO_BRIDGE_config_yaml=ON \
					-DHUMOTO_MODULE_wpg04=ON \
					-DHUMOTO_MODULE_wpg05=OFF \
					../humoto/
	cd build; ${MAKE} ${MAKE_FLAGS}

wpg05:
	mkdir -p build;
	cd build; cmake \
					-DHUMOTO_BUILD_TESTS=ON \
					-DHUMOTO_BUILD_REGRESSION_TESTS=ON \
					-DHUMOTO_MODULES_DIRS=`pwd`/../extra_modules/ \
					-DHUMOTO_BRIDGE_qpOASES=ON \
					-DHUMOTO_BRIDGE_config_yaml=ON \
					-DHUMOTO_MODULE_wpg04=OFF \
					-DHUMOTO_MODULE_wpg05=ON \
					../humoto/
	cd build; ${MAKE} ${MAKE_FLAGS}

tests: all
	cd build; ${MAKE} ${MAKE_FLAGS} test

update:
	git submodule update --init

.PHONY: build
