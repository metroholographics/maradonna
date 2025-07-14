PROJ_NAME = main.exe
C_FLAGS ?= -Wall -Wextra -std=c99
C_FILES = src/*.c
INCLUDE_PATH = libs
RAYLIB_FLAGS = -Llibs -lraylib -lopengl32 -lgdi32 -lwinmm

all:
	gcc $(C_FLAGS) -I$(INCLUDE_PATH) $(C_FILES) $(RAYLIB_FLAGS) -o $(PROJ_NAME)