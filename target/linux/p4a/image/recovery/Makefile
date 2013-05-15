#
# Copyright (C) 2008-2009 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=recovery
PKG_RELEASE:=1

PKG_BUILD_DIR:=$(KERNEL_BUILD_DIR)/$(PKG_NAME)/linux-$(LINUX_VERSION)
PKG_SOURCE:=linux-$(LINUX_VERSION).tar.bz2

include $(INCLUDE_DIR)/package.mk

define Build/Compile
	tar zxf ${CURDIR}/root-p4a.tar.gz -C $(PKG_BUILD_DIR)
	-ln -s /etc/preinit $(PKG_BUILD_DIR)/root-p4a/init
	cp ${CURDIR}/defconfig $(PKG_BUILD_DIR)/.config
	ARCH=arm $(MAKE) -C $(PKG_BUILD_DIR) CROSS_COMPILE=$(TARGET_CROSS)
endef

define Build/InstallDev
	mkdir -p $(BIN_DIR)
	$(TARGET_CROSS)objcopy -O binary -R .note -R .comment -S $(PKG_BUILD_DIR)/arch/arm/boot/compressed/vmlinux linux.bin
	mkimage -A arm -O linux -T kernel -C none -a 00008000 -e 00008000 -n "OpenWRT Kernel Image p4abu" -d linux.bin uImage

	mv uImage $(BIN_DIR)/recovery.img
	rm linux.bin
endef

$(eval $(call Build/DefaultTargets))
