#! /bin/bash

dts devel build -f -H weirdbot.local
dts devel run -H weirdbot.local -- --privileged