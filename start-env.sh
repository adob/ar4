#!/bin/bash

path=$(readlink -f -- "$0")
filename=$(basename "$path")
dir=$(dirname "$path")
name=$(basename "$dir")

ln "$HOME/.gitconfig" "$dir/.gitconfig" 2>/dev/null

# https://man7.org/linux/man-pages/man5/firejail-profile.5.html
exec firejail --profile=<(cat <<-EOF
    private $dir
    read-only ~/$filename
    read-only ~/.gitconfig

    env JAIL_USER=$name

    private-dev
    private-tmp

    # apparmor
    caps.drop all
    seccomp
    nonewprivs

    # https://github.com/netblue30/firejail/issues/3303
    # noroot

    hostname $name

    ipc-namespace
    #no3d
    #noautopulse
    #nodvd
    #nogroups
    #nosound
    #notv
    #nou2f
    #novideo
    machine-id
    blacklist /var

    name $name

    allow-debuggers

    # only supported starting in v0.9.70
    # tab

    seccomp !chroot,!mount,!pivot_root,!umount2
EOF
) "bin/devbox" shell

