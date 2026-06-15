#!/usr/bin/env bash

set -u

missing=0

check_command() {
    local label="$1"
    shift

    if command -v "$1" >/dev/null 2>&1; then
        printf 'OK      %-10s %s\n' "$label" "$(command -v "$1")"
    else
        printf 'MISSING %-10s not found in PATH\n' "$label"
        missing=1
    fi
}

check_command "git" git
check_command "python3" python3

if command -v pip >/dev/null 2>&1; then
    printf 'OK      %-10s %s\n' "pip" "$(command -v pip)"
elif command -v pip3 >/dev/null 2>&1; then
    printf 'OK      %-10s %s\n' "pip" "$(command -v pip3)"
else
    printf 'MISSING %-10s not found in PATH (checked pip and pip3)\n' "pip"
    missing=1
fi

check_command "cmake" cmake
check_command "ninja" ninja
check_command "idf.py" idf.py

if [[ -n "${IDF_PATH:-}" ]]; then
    printf 'OK      %-10s %s\n' "IDF_PATH" "$IDF_PATH"
else
    printf 'MISSING %-10s ESP-IDF environment is not active\n' "IDF_PATH"
    missing=1
fi

if (( missing != 0 )); then
    printf '\nEnvironment check failed. Source ESP-IDF export.sh and rerun.\n'
    exit 1
fi

printf '\nEnvironment check passed.\n'
