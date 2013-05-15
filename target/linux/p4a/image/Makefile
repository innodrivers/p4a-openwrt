# 
# Copyright (C) 2006-2010 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

PAGESIZE=4k
BLOCKSIZE=512k
JFFS2_BLOCKSIZE=$(BLOCKSIZE)
JFFS2OPTS += --little-endian --pagesize=0x1000 --no-cleanmarkers --pad

UBIFS_OPTS = -m 2048 -e 126KiB -c 1996 -U
UBINIZE_OPTS = -m 2048 -p 128KiB -s 512

#define Build/Clean
#	$(MAKE) -C recovery clean
#endef

#define Build/Compile
#	$(MAKE) -C recovery compile
#endef

define Image/BuildKernel
	$(TARGET_CROSS)objcopy -O binary -R .note -R .comment -S $(LINUX_DIR)/arch/arm/boot/compressed/vmlinux linux.bin
	mkimage -A arm -O linux -T kernel -C none -a 00008000 -e 00008000 -n "OpenWRT Kernel Image p4abu" -d linux.bin uImage

	mv uImage $(BIN_DIR)/$(IMG_PREFIX)-$(KERNEL)-uImage
	rm linux.bin
endef

define Image/Build/Profile/p4abu
	$(STAGING_DIR_HOST)/bin/mkinnofideifw	\
		-B p4abu -M "innofidei inc."		\
		-P kernel=$(BIN_DIR)/$(IMG_PREFIX)-$(KERNEL)-uImage -A $(PAGESIZE)	\
		-P rootfs=$(BIN_DIR)/$(IMG_PREFIX)-root.$(1) -A $(PAGESIZE)	\
		-O $(BIN_DIR)/$(IMG_PREFIX)-p4abu-kernel-$(1)-sysupgrade.bin

endef

define Image/Build/squashfs
	$(call prepare_generic_squashfs,$(KDIR)/root.squashfs)
endef

define Image/Build
	$(call Image/Build/$(1))
	$(CP) $(KDIR)/root.$(1) $(BIN_DIR)/$(IMG_PREFIX)-root.$(1)
	echo $(PROFILE)

	$(call Image/Build/Profile/$(PROFILE),$(1))
endef

$(eval $(call BuildImage))
