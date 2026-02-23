CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wno-deprecated-declarations
LDFLAGS := -lGLEW -lGL -lGLU -lglut

# Targets
TARGET1 := physicssim
TARGET2 := dynamicpaths
TARGET3 := particlePaths

# Source files
SRCS1 := sim.cpp
SRCS2 := dynamicpaths.cpp
SRCS3 := main.cpp particle.cpp pathFinding.cpp robot.cpp

# Object files
OBJS1 := $(SRCS1:.cpp=.o)
OBJS2 := $(SRCS2:.cpp=.o)
OBJS3 := particle.o pathFinding.o robot.o main.o

.PHONY: all clean

# Build all executables by default
all: $(TARGET1) $(TARGET2) $(TARGET3)

# Build the original physics simulation
$(TARGET1): $(OBJS1)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Build the dynamic pathfinding simulation
$(TARGET2): $(OBJS2)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Build the modular particle system with pathfinding
$(TARGET3): $(OBJS3)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Clean all build artifacts
clean:
	rm -f $(OBJS1) $(OBJS2) $(OBJS3) $(TARGET1) $(TARGET2) $(TARGET3)

# Pattern rule for object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<