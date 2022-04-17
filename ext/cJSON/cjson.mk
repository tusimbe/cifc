# FATFS files.
CJSONSRC = $(CHIBIOS)/ext/cJSON/cJSON.c

CJSONINC = $(CHIBIOS)/ext/cJSON

# Shared variables
ALLCSRC += $(CJSONSRC)
ALLINC  += $(CJSONINC)