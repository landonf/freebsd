/*-
 * Copyright (c) 2014 Baptiste Daroussin <bapt@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer
 *    in this position and unchanged.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR(S) ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR(S) BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#if __FreeBSD_version > 1001510
#include <sys/capsicum.h>
#else
#include <sys/capabilities.h>
#endif
#include <sys/types.h>

#include <ctype.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stringlist.h>
#include <termios.h>
#include <unistd.h>

#define C_OPTION 0x1

static StringList *includes;

static void
usage(void)
{

	fprintf(stderr, "usage: soelim [-Crtv] [-I dir] [files]\n");

	exit(EXIT_FAILURE);
}

static const char *
relpath(const char *path)
{

	while (*path == '/' && *path != '\0')
		path++;

	return (path);
}

static FILE *
soelim_fopen(int rootfd, const char *name)
{
	FILE *f = NULL;
	char path[PATH_MAX];
	size_t i;
	int fd;

	if (strcmp(name, "-") == 0)
		return (stdin);

	if ((fd = openat(rootfd, relpath(name), O_RDONLY)) != -1) {
		f = fdopen(fd, "r");
		goto out;
	}

	if (*name == '/') {
		warn("can't open '%s'", name);
		return (NULL);
	}

	for (i = 0; i < includes->sl_cur; i++) {
		snprintf(path, sizeof(path), "%s/%s", includes->sl_str[i],
		    name);
		if ((fd = openat(rootfd, relpath(path), O_RDONLY)) != -1) {
			f = fdopen(fd, "r");
			break;
		}
	}

out:
	if (f == NULL)
		warn("can't open '%s'", name);

	return (f);
}

static int
soelim_file(int rootfd, FILE *f, int flag)
{
	char *line = NULL;
	char *walk, *cp;
	size_t linecap = 0;
	ssize_t linelen;

	if (f == NULL)
		return (1);

	while ((linelen = getline(&line, &linecap, f)) > 0) {
		if (strncmp(line, ".so", 3) != 0) {
			printf("%s", line);
			continue;
		}

		walk = line + 3;
		if (!isspace(*walk) && ((flag & C_OPTION) == 0)) {
			printf("%s", line);
			continue;
		}

		while (isspace(*walk))
			walk++;

		cp = walk;
		while (*cp != '\0' && !isspace(*cp))
			cp++;
		*cp = 0;
		if (cp < line + linelen)
			cp++;

		if (*walk == '\0') {
			printf("%s", line);
			continue;
		}
		if (soelim_file(rootfd, soelim_fopen(rootfd, walk), flag) == 1) {
			free(line);
			return (1);
		}
		if (*cp != '\0')
			printf("%s", cp);
	}

	free(line);
	fclose(f);

	return (0);
}

int
main(int argc, char **argv)
{
	int ch, i, rootfd;
	int ret = 0;
	int flags = 0;
	char cwd[MAXPATHLEN];
	unsigned long cmd;
	cap_rights_t rights;

	includes = sl_init();
	if (getcwd(cwd, sizeof(cwd)) != NULL)
		sl_add(includes, cwd);

	if (includes == NULL)
		err(EXIT_FAILURE, "sl_init()");

	while ((ch = getopt(argc, argv, "CrtvI:")) != -1) {
		switch (ch) {
		case 'C':
			flags |= C_OPTION;
			break;
		case 'r':
		case 'v':
		case 't':
			/* stub compatibility with groff's soelim */
			break;
		case 'I':
			sl_add(includes, optarg);
			break;
		default:
			sl_free(includes, 0);
			usage();
		}
	}

	argc -= optind;
	argv += optind;

	rootfd = open("/", O_DIRECTORY | O_RDONLY);
	if (rootfd == -1)
		err(EXIT_FAILURE, "unable to open '/'");
	cap_rights_init(&rights, CAP_READ, CAP_FSTAT, CAP_IOCTL);
	/*
	 * EBADF in case stdin is closed by the caller
	 */
	if (cap_rights_limit(STDIN_FILENO, &rights) < 0 && errno != ENOSYS
	    && errno != EBADF)
		err(EXIT_FAILURE, "unable to limit rights for stdin");
	cap_rights_init(&rights, CAP_WRITE, CAP_FSTAT, CAP_IOCTL);
	if (cap_rights_limit(STDOUT_FILENO, &rights) < 0 && errno != ENOSYS)
		err(EXIT_FAILURE, "unable to limit rights for stdout");
	if (cap_rights_limit(STDERR_FILENO, &rights) < 0 && errno != ENOSYS)
		err(EXIT_FAILURE, "unable to limit rights for stderr");
	cap_rights_init(&rights, CAP_READ, CAP_LOOKUP, CAP_FSTAT, CAP_FCNTL);
	if (cap_rights_limit(rootfd, &rights) < 0 && errno != ENOSYS)
		err(EXIT_FAILURE, "unable to limit rights");

	cmd = TIOCGETA;
	if (cap_ioctls_limit(STDOUT_FILENO, &cmd, 1) < 0 && errno != ENOSYS)
		err(EXIT_FAILURE, "unable to limit ioctls for stdout");
	if (cap_ioctls_limit(STDERR_FILENO, &cmd, 1) < 0 && errno != ENOSYS)
		err(EXIT_FAILURE, "unable to limit ioctls for stderr");
	if (cap_ioctls_limit(STDIN_FILENO, &cmd, 1) < 0 && errno != ENOSYS)
		err(EXIT_FAILURE, "unable to limit ioctls for stdin");

	if (cap_enter() < 0 && errno != ENOSYS)
		err(EXIT_FAILURE, "unable to enter capability mode");

	if (argc == 0)
		ret = soelim_file(rootfd, stdin, flags);

	for (i = 0; i < argc; i++)
		ret = soelim_file(rootfd, soelim_fopen(rootfd, argv[i]), flags);

	sl_free(includes, 0);
	close(rootfd);

	return (ret);
}
