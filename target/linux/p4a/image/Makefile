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

define Build/Clean
endef

define Build/Compile
endef

define Image/BuildRecovery
	cp ${LINUX_DIR}/.config $(TMP_DIR)/.config.before_build_recovery
	cp ${LINUX_DIR}/arch/arm/boot/zImage $(TMP_DIR)/.zImage.before_build_recovery

	$(if $(CONFIG_TARGET_ROOTFS_INITRAMFS),,cp recovery.init $(TARGET_DIR)/init)
	echo 'CONFIG_BLK_DEV_INITRD=y' >> $(LINUX_DIR)/.config
	echo 'CONFIG_INITRAMFS_SOURCE="$(strip $(TARGET_DIR) $(INITRAMFS_EXTRA_FILES))"' >> $(LINUX_DIR)/.config
	echo 'CONFIG_INITRAMFS_ROOT_UID=$(shell id -u)' >> $(LINUX_DIR)/.config
	echo 'CONFIG_INITRAMFS_ROOT_GID=$(shell id -g)' >> $(LINUX_DIR)/.config
	echo 'CONFIG_INITRAMFS_COMPRESSION_NONE=y' >> $(LINUX_DIR)/.config
	echo -e '# CONFIG_INITRAMFS_COMPRESSION_GZIP is not set\n# CONFIG_RD_GZIP is not set)' >> $(LINUX_DIR)/.config
	echo -e '# CONFIG_INITRAMFS_COMPRESSION_BZIP2 is not set\n# CONFIG_RD_BZIP2 is not set)' >> $(LINUX_DIR)/.config
	echo -e '# CONFIG_INITRAMFS_COMPRESSION_LZMA is not set\n# CONFIG_RD_LZMA is not set)' >> $(LINUX_DIR)/.config
	echo -e '# CONFIG_INITRAMFS_COMPRESSION_LZO is not set\n# CONFIG_RD_LZO is not set)' >> $(LINUX_DIR)/.config

	$(MAKE) -C $(LINUX_DIR) CROSS_COMPILE=$(TARGET_CROSS) ARCH="$(LINUX_KARCH)"
	cp $(LINUX_DIR)/arch/arm/boot/zImage $(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-recovery.img
	$(if $(CONFIG_TARGET_ROOTFS_INITRAMFS),,rm -f $(TARGET_DIR)/init)

	mv $(TMP_DIR)/.zImage.before_build_recovery $(LINUX_DIR)/arch/arm/boot/zImage
	mv $(TMP_DIR)/.config.before_build_recovery $(LINUX_DIR)/.config
endef

define Image/BuildKernel
	cp $(LINUX_DIR)/arch/arm/boot/zImage $(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-$(KERNEL)-zImage

	$(STAGING_DIR_HOST)/bin/mkinnofideifw	\
		-B $(PROFILE) -M "innofidei inc."		\
		-P kernel=$(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-$(KERNEL)-zImage -A $(PAGESIZE)	\
		-O $(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-kernel-sysupgrade.bin

	$(call Image/BuildRecovery)

	$(STAGING_DIR_HOST)/bin/mkinnofideifw	\
		-B $(PROFILE) -M "innofidei inc."		\
		-P recovery=$(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-recovery.img -A $(PAGESIZE)	\
		-O $(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-recovery-sysupgrade.bin
endef

define Image/Build/InnofideiFW
	$(STAGING_DIR_HOST)/bin/mkinnofideifw	\
		-B $(PROFILE) -M "innofidei inc."		\
		-P rootfs=$(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-root.$(1) -A $(PAGESIZE)	\
		-O $(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-$(1)-sysupgrade.bin

	$(STAGING_DIR_HOST)/bin/mkinnofideifw	\
		-B $(PROFILE) -M "innofidei inc."		\
		-P kernel=$(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-$(KERNEL)-zImage -A $(PAGESIZE)	\
		-P rootfs=$(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-root.$(1) -A $(PAGESIZE)	\
		-O $(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-kernel-$(1)-sysupgrade.bin

	$(STAGING_DIR_HOST)/bin/mkinnofideifw	\
		-B $(PROFILE) -M "innofidei inc."		\
		-P kernel=$(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-$(KERNEL)-zImage -A $(PAGESIZE)	\
		-P rootfs=$(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-root.$(1) -A $(PAGESIZE)	\
		-P recovery=$(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-recovery.img -A $(PAGESIZE)	\
		-O $(BIN_DIR)/$(IMG_PREFIX)-$(PROFILE)-kernel-$(1)-recovery-sysupgrade.bin
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
