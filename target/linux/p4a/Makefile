#
# Copyright (C) 2006-2011 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk

ARCH:=arm
BOARD:=p4a
BOARDNAME:=Innofidei p4a
FEATURES:=squashfs

LINUX_VERSION:=2.6.37.1

define Target/Description
	Build firmware images for Innofidei P4A based routers
endef

include $(INCLUDE_DIR)/target.mk

$(eval $(call BuildTarget))
