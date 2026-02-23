CXX := g++
CXXFLAGS := -std=c++17 -O3 -march=native -Wall -Wextra -Wno-deprecated-declarations
LDFLAGS := -lGLEW -lGL -lGLU -lglut -pthread

# Targets
TARGET1 := physicssim
TARGET2 := dynamicpaths
TARGET3 := plannersim.bin
SCRIPT3 := plannersim

# Source files
SRCS1 := sim.cpp
SRCS2 := dynamicpaths.cpp
SRCS3 := main.cpp particle.cpp pathFinding.cpp robot.cpp gpu_renderer.cpp

# Object files
OBJS1 := $(SRCS1:.cpp=.o)
OBJS2 := $(SRCS2:.cpp=.o)
OBJS3 := particle.o pathFinding.o robot.o main.o gpu_renderer.o

.PHONY: all clean

# Build all executables by default
all: $(TARGET1) $(TARGET2) $(SCRIPT3)

# Generate the plannersim launcher script
$(SCRIPT3): $(TARGET3)
	@printf '#!/usr/bin/env sh\n# Auto-generated GPU launcher — edit Makefile to change\nexport MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA\nexport __NV_PRIME_RENDER_OFFLOAD=1\nexport __GLX_VENDOR_LIBRARY_NAME=nvidia\nexec "$$(dirname "$$0")/$(TARGET3)" "$$@"\n' > $@
	@chmod +x $@
	@echo "Generated ./$@ wrapper"

# Build the original physics simulation
$(TARGET1): $(OBJS1)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Build the dynamic pathfinding simulation
$(TARGET2): $(OBJS2)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Build the modular particle system with pathfinding
$(TARGET3): $(OBJS3)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
	@rm -f $(SCRIPT3)  # force script regeneration on binary rebuild

# Clean all build artifacts
clean:
	rm -f $(OBJS1) $(OBJS2) $(OBJS3) $(TARGET1) $(TARGET2) $(TARGET3) $(SCRIPT3)

# Pattern rule for object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<