#!/usr/bin/env bash

NO_FORMAT="\033[0m"
F_BOLD="\033[1m"
C_LIME="\033[38;5;10m"
C_RED="\033[38;5;9m"

for SCRIPT in ./scripts/*; do
    if bash $SCRIPT; then
        echo -e "${F_BOLD}${C_LIME}$SCRIPT ran successfully!${NO_FORMAT}"
    else
        echo -e "${F_BOLD}${C_RED}$SCRIPT ERRORED with status code $?!${NO_FORMAT}"
    fi
done
