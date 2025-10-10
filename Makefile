CXX := clang++
CXXFLAGS := -std=c++17 -Wall -Wextra -I/opt/homebrew/include -Wno-deprecated-declarations
LDFLAGS := -L/opt/homebrew/lib -lGLEW -framework OpenGL -framework GLUT

# Targets
TARGET1 := physicssim
TARGET2 := dynamicpaths

# Source files
SRCS1 := sim.cpp
SRCS2 := dynamicpaths.cpp

# Object files
OBJS1 := $(SRCS1:.cpp=.o)
OBJS2 := $(SRCS2:.cpp=.o)

.PHONY: all clean

# Build both executables by default
all: $(TARGET1) $(TARGET2)

# Build the original physics simulation
$(TARGET1): $(OBJS1)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Build the dynamic pathfinding simulation
$(TARGET2): $(OBJS2)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Clean all build artifacts
clean:
	rm -f $(OBJS1) $(OBJS2) $(TARGET1) $(TARGET2)

# Pattern rule for object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<