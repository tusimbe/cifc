# List of all the board related files.
BOARDSRC = $(CHIBIOS)/config/board/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/config/board

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
