#!/usr/bin/env bash
set -e

# Default configs
TARGET="all"
DEBUG=0
GENERATE_LST=0

# Parse args
while [[ $# -gt 0 ]]; do
    case "$1" in
        build)
            TARGET="all"
            shift
            ;;
        clean)
            TARGET="clean"
            shift
            ;;
        erase)
            TARGET="erase"
            shift
            ;;
        debug)
            DEBUG=1
            shift
            ;;
        lst)
            GENERATE_LST=1
            shift
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

# Debug: show build configuration
echo "================================"
echo " Target             : $TARGET"
echo " Debug              : $DEBUG"
echo " GENERATE_LST       : $GENERATE_LST"
echo "================================"

# Call make
make $TARGET DEBUG=$DEBUG GENERATE_LST=$GENERATE_LST "${EXTRA_ARGS[@]}"
