LIB = liblsm


OBJECTS += \
  gyro.o \

ifeq ($(TOOLCHAIN),$(filter $(TOOLCHAIN), gcc coatigcc))
$(info toolchain = gcc)
DEPS += libmspware \


else
$(info lib_root = $(LIB_ROOT))
CFLAGS += -I$(LIB_ROOT)/libmspware/src/include \

LFLAGS += -L$(LIB_ROOT)/libmspware/bld/gcc \

LIBS += -lmspware \

endif # TOOLCHAIN

DEPS += \
  libmsp \
  libio\


override SRC_ROOT = ../../src

override CFLAGS += \
  -I$(SRC_ROOT)/include \
	-I$(SRC_ROOT)/include/$(LIB) \
