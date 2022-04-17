# FATFS files.
LFSSRC = $(CHIBIOS)/ext/littlefs/lfs.c \
         $(CHIBIOS)/ext/littlefs/lfs_util.c

LFSINC = $(CHIBIOS)/ext/littlefs

# Shared variables
ALLCSRC += $(LFSSRC)
ALLINC  += $(LFSINC)
