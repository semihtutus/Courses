# Compiler and flags
CC = gcc
CFLAGS = -std=c23 -Wall -Werror -g
LDFLAGS = 
TARGET = battleworm
SRC = battleworm.c

# Default target
all: $(TARGET)

# Compile the target
$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

# Memory check with Valgrind
memcheck: $(TARGET)
	valgrind --tool=memcheck --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file=battleworm_log.txt ./$(TARGET)

# Clean
clean:
	rm -f $(TARGET) battleworm_log.txt

# Run
run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run memcheck