#
# Copyright (C) 2006-2010 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/p4abu
  NAME:=p4abu
  PACKAGES :=
endef

define Profile/p4abu/Description
	Package set optimized for P4A bring-up board.
endef

$(eval $(call Profile,p4abu))
