all:
	cmake -Bbuild -H. -D CMAKE_BUILD_TYPE=Debug
	cmake --build build -j4

clean:
	cmake --build build/ --target clean