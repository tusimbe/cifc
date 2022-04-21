CRASHCATCHERASM = $(CHIBIOS)/ext/CrashCatcher/CrashCatcher_armv7m.S

CRASHCATCHERSRC = $(CHIBIOS)/ext/CrashCatcher/CrashCatcher.c  \
                  $(CHIBIOS)/ext/CrashCatcher/HexDump.c       \
				  $(CHIBIOS)/ext/CrashCatcher/binding.c  

CRASHCATCHERINC = $(CHIBIOS)/ext/CrashCatcher

# Shared variables
ALLXASMSRC += $(CRASHCATCHERASM)
ALLCSRC    += $(CRASHCATCHERSRC)
ALLINC     += $(CRASHCATCHERINC)