
# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2

# Target executable name
TARGET = test_simtemp

# Source files
SRCS = main.cpp

# Default rule
all: $(TARGET)

# Link and compile
$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS)

# Clean up build artifacts
clean:
	rm -f $(TARGET)

# Optional: run the program
run: $(TARGET)
	./$(TARGET)
