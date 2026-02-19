.PHONY: all clean

all:
	cmake -S . -B build
	cmake --build build -j4

clean:
	@if [ -d build ]; then \
		cmake --build build --target clean; \
	else \
		echo "No build/ directory found; nothing to clean."; \
	fi
