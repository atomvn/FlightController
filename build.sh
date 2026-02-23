#!/usr/bin/env bash

case "$1" in
    build) make ;;
    flash) make flash ;;
    clean) make clean ;;
    *) echo "Usage: $0 {build|flash|clean}" ;;
esac
