LCM = lcm-gen
LCMTYPES_NAMES := $(shell ls *.lcm)

# Construct the pathnames to the C types
LCMTYPES_C := $(LCMTYPES_NAMES:%.lcm=%.c)
LCMTYPES_C := $(addprefix lcmtypes_c/,$(LCMTYPES_C))
LCMTYPES_O = $(LCMTYPES_C:%.c=%.o)
CC = gcc
CFLAGS = -g `pkg-config --cflags lcm`

# Python build options
PY_PACKAGE = lcmtypes_py
LCMTYPES_PY := $(LCMTYPES_NAMES:%.lcm=%.py)
LCMTYPES_PY := $(addprefix lcmtypes_py/,$(LCMTYPES_PY))

# Java build options
BUILD_PATH = lcmtypes_java
LCMTYPES_JAVA := $(LCMTYPES_NAMES:%.lcm=%.java)
LCMTYPES_JAVA := $(addprefix $(BUILD_PATH)/src/lcmtypes/,$(LCMTYPES_JAVA))
LCM_JAR = /usr/local/share/java/lcm.jar

ALL = $(LCMTYPES_C) $(LCMTYPES_O) $(LCMTYPES_PY)
ifneq "$(shell which javac)" ""
ALL += lcmtypes.jar
endif

all: $(ALL)

lcmtypes_c/%.c: %.lcm
	mkdir -p lcmtypes_c
	$(LCM) -c --c-typeinfo --c-cpath lcmtypes_c/ --c-hpath lcmtypes_c/ $<

lcmtypes_c/%.o: lcmtypes_c/%.c
	mkdir -p lcmtypes_c
	$(CC) $(CFLAGS) -c $^ -o $@

clean:
	rm -rf build/ lcmtypes_java/ lcmtypes_py/ lcmtypes_c/
	rm -f *.c *.h *.o *~ lcmtypes.jar
	rm -rf *~ lcmtypes/* *.pyc

$(BUILD_PATH)/src/lcmtypes/%.java: %.lcm
	$(LCM) -j --jpath=$(BUILD_PATH)/src --jdefaultpkg=lcmtypes $<

lcmtypes.jar: $(LCMTYPES_JAVA)
	mkdir -p $(BUILD_PATH)/class
	javac -cp $(LCM_JAR) -d $(BUILD_PATH)/class $^
	jar cf $@ -C $(BUILD_PATH)/class .

lcmtypes_py/%.py: %.lcm
	@$(LCM) -p --package-prefix $(PY_PACKAGE) $<


