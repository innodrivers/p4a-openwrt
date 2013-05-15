

# upgrade_info name
fname_upgrade_info="upgrade_info"

# decompress upgrade_info into specific directory
fname_upgrade_info_prefix="/tmp"

# upgrade_info path after decompress
dl_upgrade_info="$fname_upgrade_info_prefix/$fname_upgrade_info"


# get ini config file item value
# Usage: cfg_get <config file> <section> <key>
cfg_get() {
	local val
	val=`awk -F '=' '/\['$2'\]/{a=1}a==1&&$1~/'$3'/{print $2;exit}' $1`
	echo $val 
}

# get current board name
board_name() {
	local machine

    machine=$(awk 'BEGIN{FS="[ \t]+:[ \t]"} /Hardware/ {print $2}' /proc/cpuinfo)
	echo $machine
}

# Usage: platform_check_image <firmware file>
platform_check_image() {
	echo "platform check image" "$ARGV"
	local fwfile="$1"
	local magic="$(get_image "$fwfile" "cat" | dd bs=2 count=1 2>/dev/null | hexdump -n 2 -e '1/1 "%02x"')"

	# check file format is .tgz
	[ "$magic" != "1f8b" ] && {
		echo "Invalid image type."
		return 1
	}

	# firmware description file exist ?
	get_image "$fwfile" "cat" | tar zt | grep -qs $fname_upgrade_info  || {
		echo "firmware upgrade description file ($fname_upgrade_info) not exist."
		return 1
	}

	# decompress the firmware description file
	get_image "$fwfile" "cat" | tar zx "$fname_upgrade_info" -C "$fname_upgrade_info_prefix" || {
		echo "decompress $fname_upgrade_info failed."
		return 1
	}
	
	local target_board=$(cfg_get "$dl_upgrade_info" "COMMON" "board")
	local local_board=$(board_name)

	# check  board name
	[ "$local_board" != "$target_board" ] && {
		echo "the firmware not support for this board"
		return 1
	}

	return 0
}

# Usage : _upgrade_common <firmware file> <part name>
_upgrade_common() {
	local fwfile="$1"
	local part="$2"
	local imgfile=$(cfg_get "$dl_upgrade_info" "IMAGES" "$part")

	[ -z "$imgfile" ] && {
		echo "omit upgrade $part"
		return 0
	}

	# check part exist
	grep -qs "$part" /proc/mtd || {
		echo "partition $part not exist, so abort upgrade $part"
		return 0
	}

	echo "prepare upgrade $part" 

	get_image "$fwfile" "cat" | tar -zx "$imgfile" -O | mtd write - "$part"	
}

platform_do_upgrade() {
	local fwfile="$1"
	local part_list="ps dsp kernel rootfs recovery"

	for part in $part_list; do
		_upgrade_common "$fwfile" "$part"	|| {
			echo "upgrade $part failed, stop!"
			break
		}
	done
}

disable_watchdog() {
	echo "disable watchdog"
}

append sysupgrade_pre_upgrade disable_watchdog
