PROJECT = RM18A

# Imported source files and paths
COREDIR = ./RM18A-ChibiOS18-Core

# UDEFS = -DNO_APP_CONFIG

# Compile all .c and .cpp files in the root directory
ALLCSRC += $(wildcard ./*.c)
ALLCPPSRC += $(wildcard ./*.cpp)

include $(COREDIR)/core.mk
