# Compiler and flags
CC = gcc
CFLAGS = -std=c99 -Wall -Werror -g
LDFLAGS = 
TARGET = cube
SRC = cube.c

# Default target
all: $(TARGET)

# Compile the target
$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

# Memory check with Valgrind
memcheck: $(TARGET)
	valgrind --tool=memcheck --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file=cube_log.txt ./$(TARGET)

# Clean
clean:
	rm -f $(TARGET) cube_log.txt cube_result.stl

# Run
run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run memcheck