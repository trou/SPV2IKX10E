#!/bin/sh

set -e

usage() {
    cat << EOF
usage: bsh-wificonnect.sh [--] ssid passphrase
       bsh-wificonnect.sh -l

Connect to a protected WiFi access point

positional arguments:
  ssid         network name
  passphrase   pre shared key of the network

optional arguments:
  -l           list available SSIDs and exit
EOF
    exit 1
}

main() {
    config_dir=/var/lib/connman
    config="$config_dir/predefinedWifi.config"
    temp_config="$config_dir/.bsh-wificonnect.$$"

    [ "$1" = "-l" ] && list
    [ "$1" = "--" ] && shift
    [ "$#" -eq 2 ] || usage >&2

    ssid="$1"
    passphrase="$2"

    scan
    if ! is_ssid_available; then
        >&2 echo "SSID not available: $ssid"
        exit 1
    fi

    trap 'rm -f "$config_dir"/.bsh-wificonnect.*' INT QUIT HUP TERM EXIT
    generate_config > "$temp_config"
    sync "$temp_config"
    mv "$temp_config" "$config"
}

list() {
    scan
    iwlist scan 2>/dev/null | grep ESSID
    exit 0
}

scan() {
    # error can be ignored
    connmanctl enable wifi 2>/dev/null
    connmanctl scan wifi
}

is_ssid_available() {
    ! connmanctl services | awk \
        'substr($0, 5, length("'"$ssid"'")) == "'"$ssid"'" { exit 1 }'
}

generate_config() {
    cat << EOF
[global]
Name = Predefined Wifi Network
Description = Connect wifi to a predefined router

[service_PredefinedRouter]
Type = wifi
Security = psk
# Name = $ssid
SSID = $(hex_ssid)
# Passphrase = $passphrase
Passphrase = $(raw_key)
IPv4 = dhcp
EOF
}

hex_ssid() {
    printf "%s" "$ssid" | hexdump -ve '/1 "%02x"'
}

raw_key() {
    wpa_passphrase "$ssid" "$passphrase" \
        | awk -F= '$1 ~ /^\s*psk$/ { print $2 }'
}

main "$@"
