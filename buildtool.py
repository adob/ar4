#!/usr/bin/env python3

from deps.buildtool import *

INCPATH += [
    "-Ideps/baselib", "-iquotelib", "-iquotedeps"
]

main()
