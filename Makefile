PILOTS = AcTeamMobile

CC = gcc
CFLAGS = -Wall -Wextra -ansi -pedantic -Wpedantic -g -Iinclude
LDLIBS = -lm

SANITIZE=off
ifeq ($(SANITIZE),on)
CFLAGS += -fsanitize=address
endif

.PHONY: all clean distclean install try

DRIVERS_PATH = ../drivers
SRC_DIR = src
OBJ_DIR = object
INC_DIR = include

OBJS = $(OBJ_DIR)/AcTeamMobile.o $(OBJ_DIR)/raycasting.o $(OBJ_DIR)/logic.o

all: $(PILOTS)

install: $(PILOTS)
	mkdir -p $(DRIVERS_PATH)
	cp $(PILOTS) $(DRIVERS_PATH)

try: install
	cd .. ; ./GrandPrix

$(PILOTS): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

distclean: clean
	rm -f $(PILOTS)

clean:
	rm -rf $(OBJ_DIR) *~ $(SRC_DIR)/*~ $(INC_DIR)/*~
