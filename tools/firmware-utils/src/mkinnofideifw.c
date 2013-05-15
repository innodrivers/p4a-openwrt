/*
 *  Copyright (C) 2013 Innofidei Inc.
 *
 *  Author : Jimmy.li <lizhengming@innofidei.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>     /* for unlink() */
#include <libgen.h>
#include <getopt.h>     /* for getopt() */
#include <stdarg.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define _ALIGN_UP(x, size)		(((x) + ((size)-1)) / (size) * (size))

#define WORKDIR		"/tmp/mkinnofideifw"
#define UPGRADE_INFO_FILE	"upgrade_info"

#define MAX_PARTS			(8)
#define MAX_PART_NAME_LEN	(32)
#define MAX_IMAGE_NAME_LEN	(256)

static char* board;
static char* manufacturer;
static char* part_img[MAX_PARTS];
static unsigned long part_img_alignsz[MAX_PARTS];
static char* output_file;

static struct {
	char part[MAX_PART_NAME_LEN];
	char image[MAX_IMAGE_NAME_LEN];
	unsigned long alignsz;
} part_info[MAX_PARTS];

static int part_count;

static char* progname;

static unsigned long long memparse(const char* ptr, char **retptr)
{
	char* endptr;
	unsigned long long ret = strtoull(ptr, &endptr, 0);

	switch (*endptr) {
	case 'G':
	case 'g':
		ret <<= 10;
	case 'M':
	case 'm':
		ret <<= 10;
	case 'K':
	case 'k':
		ret <<= 10;
		endptr++;
	default:
		break;
	}

	if (retptr)
		*retptr = endptr;
	
	return ret;
}

static int parse_and_check_images(void)
{
	int i;
	char* p;

	for (i=0; i<part_count; i++) {
		p = strchr(part_img[i], '=');
		if (p == NULL) {
			printf("invalid part-img format : %s\n", part_img[i]);
			return -1;
		}
		
		// get partition name and image name
		memset(part_info[i].part, 0, sizeof(part_info[i].part));
		memcpy(part_info[i].part, part_img[i], p - part_img[i]);

		memset(part_info[i].image, 0, sizeof(part_info[i].image));
		memcpy(part_info[i].image, p+1, strlen(part_img[i]) - (p - part_img[i] + 1));
		
		part_info[i].alignsz = part_img_alignsz[i]; 	

		// check image exist?
		if (access(part_info[i].image, R_OK | F_OK)) {
			printf("image %s not exist or cannot readable!\n", part_info[i].image);
			return -1;
		}
	}

	return 0;
}

static int create_workdir(void)
{
	int ret;

	if (access(WORKDIR, F_OK)) {
		printf("directory %s not exist!\n", WORKDIR);
		ret = mkdir(WORKDIR, 0777);

		if (ret < 0) {
			printf("cannot create %s directory\n", WORKDIR);
		}
		return ret;
	}

	return 0;
}

static int create_upgrade_info(void)
{
	FILE* f;
	char buf[512];
	int i;

	snprintf(buf, 512, "%s/%s", WORKDIR, UPGRADE_INFO_FILE);
	
	f = fopen(buf, "w+");
	if (NULL == f) {
		printf("cannot create %s\n", buf);
		return -1;
	}

	fputs("#\n# Innofidei MiFi Firmware Upgrade Description\n#\n", f);
	fputs("\n[COMMON]\n", f);
	snprintf(buf, 512, "board = %s\n", board);
	fputs(buf, f);
	snprintf(buf, 512, "manufacturer = %s\n", manufacturer);
	fputs(buf, f);

	fputs("\n[IMAGES]\n", f);
	fputs("# <partition name> = <image name>\n", f);

	for (i=0; i<part_count; i++) {
		snprintf(buf, 512, "%s = %s\n", part_info[i].part, basename(part_info[i].image));
		fputs(buf, f);
	}
	fclose(f);
	return 0;
}

static int copy_file(const char* dstfile, const char* srcfile, int alignsz, unsigned char fill_pattern)
{
	int ifd, ofd;
	char* buf;
	int bufsz = 2048;
	int ret;
	int ifsz;
	unsigned char pattern = fill_pattern;
	int padsz = 0;

	ifd = open(srcfile, O_RDONLY);
	if (ifd < 0) {
		printf("cannot open %s\n", srcfile);
		return -1;
	}

	ifsz = lseek(ifd, 0, SEEK_END);
	lseek(ifd, 0, SEEK_SET);
	
	if (alignsz > 0) {
		padsz = _ALIGN_UP(ifsz, alignsz) - ifsz;
	}

	ofd = open(dstfile, O_RDWR | O_CREAT | O_TRUNC, 0666);
	if (ofd < 0) {
		printf("cannot create %s\n", dstfile);
		close(ifd);
		return -1;
	}

	buf = (char*)malloc(bufsz);
	if (NULL == buf) {
		close(ofd);
		close(ifd);
		return -1;
	}

	while (1) {
		ret = read(ifd, buf, bufsz);
		if (ret == 0) {
			//EOF
			break;
		} else if (ret < 0) {
			perror("read fail\n");
			goto out;
		}

		ret = write(ofd, buf, ret);
		if (ret < 0) {
			perror("write fail\n");
			goto out;
		}
	}

	while (padsz) {
		int count;

		count = padsz < bufsz ? padsz : bufsz;
		memset(buf, pattern, count);
		ret = write(ofd, buf, count);
		if (ret < 0) {
			perror("write fail\n");
			goto out;
		}

		padsz -= ret;
	}
	ret = 0;

out:
	free(buf);
	close(ofd);
	close(ifd);
	return ret;
}

static int copy_images(void)
{
	int i;
	int ret;
	char dstfile[512];

	for (i=0; i<part_count; i++) {
		snprintf(dstfile, 512, "%s/%s", WORKDIR,  basename(part_info[i].image));
		ret = copy_file(dstfile, part_info[i].image, part_info[i].alignsz, 0xff);
		if (ret < 0)
			return -1;
	}

	return 0;
}

static int generate_firmware_bin(void)
{
	char command[512];
	int slen;
	int i;

	snprintf(command, 512, "tar -zcf %s -C %s %s", output_file, WORKDIR, UPGRADE_INFO_FILE);
	slen = strlen(command);

	for (i=0; i<part_count; i++) {
		snprintf(command + slen, 512 - slen, " %s", basename(part_info[i].image));
		slen = strlen(command);
	}

	printf("%s\n", command);

	return execl("/bin/sh", "sh", "-c", command, (char *) NULL);
}

static void usage(int status)
{
	FILE *stream = (status != EXIT_SUCCESS) ? stderr : stdout;

	fprintf(stream, "Usage: %s [OPTIONS...]\n", progname);
	fprintf(stream,
"\n"
"Options:\n"
"  -B <board>          create image for the board specified with <board>\n"
"  -M <manufacturer>   manufacturer name\n"
"  -P <part=image>     specified partition and image to be upgraded\n"
"  -A <align size>     specified image align size\n"
"  -O <fw file>        specified output firmware file\n"
"  -h                  show this screen\n"
"\n"
"Example:\n"
"%s -B p4abu -M \"innofidei inc.\" -P kernel=uImage.bin -A 2k -P rootfs=rootfs.squashfs -A 2k -O=firmware.img\n",
	progname);

	exit(status);
}

int main(int argc, char *argv[])
{
	int res = EXIT_FAILURE;
	int ret;
	int err;
	int part_id = -1;

	progname = basename(argv[0]);

	while ( 1 ) {
		int c;

		c = getopt(argc, argv, "B:M:P:A:O:h");
		if (c == -1)
			break;

		switch (c) {
		case 'B':
			board = optarg;
			break;
		case 'M':
			manufacturer = optarg;
			break;
		case 'P':
			part_id++;
			part_img[part_id] = optarg;
			part_img_alignsz[part_id] = 0; 
			break;
		case 'A':
			part_img_alignsz[part_id] = (int)memparse(optarg, NULL); 
			break;
		case 'O':
			output_file = optarg;
			break;
		case 'h':
			usage(EXIT_SUCCESS);
			break;
		default:
			usage(EXIT_FAILURE);
			break;
		}
	}

	if (board == NULL) {
		printf("no board specified\n");
		goto err;
	}

	if (manufacturer == NULL) {
		printf("no manufacturer specified\n");
		goto err;
	}

	if (output_file == NULL) {
		printf("no output file specified\n");
		goto err;
	}

	part_count = part_id + 1;

	ret = parse_and_check_images();
	if (ret < 0)
		goto err;

	ret = create_workdir();
	if (ret < 0)
		goto err;

	ret = copy_images();
	if (ret < 0)
		goto err;

	ret = create_upgrade_info();
	if (ret < 0)
		goto err;

	ret = generate_firmware_bin();
	if (ret < 0)
	  goto err;

	res = 0;

 err:
	return res;
}
