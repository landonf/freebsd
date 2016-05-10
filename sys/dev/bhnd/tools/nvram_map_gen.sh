#!/bin/sh

# Use C locale to ensure AWK string comparisons always produce
# a stable sort order.

# $FreeBSD$

BHND_TOOLDIR="$(dirname $0)/"

if [ -z "$AWK" ]; then
    AWK=/usr/bin/awk
fi

LC_ALL=C; export LC_ALL

"$AWK" -f "$BHND_TOOLDIR/nvram_map_gen.awk" $@
