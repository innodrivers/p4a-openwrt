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
FEATURES:=squashfs jffs2 ubifs targz
SUBTARGETS:=standalone dual

LINUX_VERSION:=2.6.37.1
CFLAGS:=-Os -pipe -march=armv7-a -mfpu=vfpv3-d16 -mfloat-abi=softfp

define Target/Description
	Innofidei WarpDrive5100 (P4A).
endef

include $(INCLUDE_DIR)/target.mk

$(eval $(call BuildTarget))
