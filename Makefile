.PHONY: python
python:
	$(MAKE) -C python install

.PHONY: cpp
cpp:
	cmake -G Ninja -S cpp -B build_cpp \
		-DCMAKE_BUILD_TYPE=Release \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
		-DCMAKE_POSITION_INDEPENDENT_CODE=ON \
		-DRKO_LIO_FETCH_CONTENT_DEPS=ON \
		-DRKO_LIO_BUILD_ROS=OFF
	cmake --build build_cpp
	touch build_cpp/COLCON_IGNORE

cpp-install: cpp
	cmake --install build_cpp --prefix install

.PHONY: clean
clean:
	rm -rf build_cpp install

.PHONY: clean_all
clean_all:
	rm -rf build build_cpp python/build install
