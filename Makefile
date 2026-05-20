PILOTS = AcTeamMobileFaboobs AcTeamElMobile AcTeamElMobileVroum

CC = gcc
CFLAGS = -Wall -Wextra -ansi -pedantic -Wpedantic -g
LDLIBS = -lm

SANITIZE=off
ifeq ($(SANITIZE),on)
CFLAGS += -fsanitize=address
endif

.PHONY: all clean distclean install try

DRIVERS_PATH = ../drivers

all: $(PILOTS)

install: $(PILOTS)
	cp $(PILOTS) $(DRIVERS_PATH)

try: install
	cd .. ; ./GrandPrix

# Règle générique pour compiler chaque pilote à partir de son fichier .c
%: %.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

distclean: clean
	rm -f $(PILOTS)

clean:
	rm -f *~ *.o