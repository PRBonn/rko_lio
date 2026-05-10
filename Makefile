.PHONY: install
install:
	pip install -v ".[all]"

.PHONY: editable
editable:
	pip install scikit-build-core pyproject_metadata pathspec pybind11 ninja cmake && pip install --no-build-isolation -ve  ".[all]"

.PHONY: core
core:
	cmake -G Ninja -S . -B build/core \
		-DCMAKE_BUILD_TYPE=Release \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
		-DCMAKE_POSITION_INDEPENDENT_CODE=ON \
		-DRKO_LIO_FETCH_CONTENT_DEPS=ON \
		-DRKO_LIO_BUILD_ROS=OFF \
		-DRKO_LIO_BUILD_TESTS=ON
	cmake --build build/core
	touch build/COLCON_IGNORE

.PHONY: core-test
core-test: core
	ctest --test-dir build/core --output-on-failure

.PHONY: clean
clean:
	rm -rf build


