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
	cp $(LINUX_DIR)/arch/arm/boot/zImage $(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-$(KERNEL)-zImage
endef

define Image/Build/InnofideiFW
	$(STAGING_DIR_HOST)/bin/mkinnofideifw	\
		-B $(PROFILE) -M "innofidei inc."		\
		-P kernel=$(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-$(KERNEL)-zImage -A $(PAGESIZE)	\
		-P rootfs=$(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-root.$(1) -A $(PAGESIZE)	\
		-O $(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-kernel-$(1)-sysupgrade.bin

endef

define Image/Build/squashfs
	$(call prepare_generic_squashfs,$(KDIR)/root.squashfs)
endef

define Image/Build
	$(call Image/Build/$(1))
	$(CP) $(KDIR)/root.$(1) $(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-root.$(1)

	$(call Image/Build/InnofideiFW,$(1))
endef

$(eval $(call BuildImage))
