include $(TOPDIR)/config.mk

LIB     := $(obj)libdwc3.o

COBJS-$(CONFIG_USB_DWC3) += core.o gadget.o ep0.o
COBJS-$(CONFIG_USB_DWC3) += misc.o

COBJS   := $(COBJS-y)
SRCS    := $(COBJS:.o=.c)
OBJS    := $(addprefix $(obj),$(COBJS))

all:    $(LIB)

$(LIB): $(obj).depend $(OBJS)
	$(call cmd_link_o_target, $(OBJS))

include $(SRCTREE)/rules.mk
sinclude $(obj).depend
