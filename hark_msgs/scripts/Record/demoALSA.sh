#!/bin/sh

if [ $# -ne 3 ]; then
  echo "ALSA recording script." 1>&2
  echo 1>&2
  echo "demoALSA1ch.sh arg1 arg2 arg3:" 1>&2
  echo "  [arg1/int]    number of frames you wan to get (100 frames/sec)" 1>&2
  echo "  [arg2/int]    number of channels" 1>&2
  echo "  [arg3/string] device name (e.g., plughw:1,0)" 1>&2
  echo 1>&2
  exit 1
fi

export BIN=n-files

echo batchflow ${BIN}/demoALSA.n $1 $2 $3
batchflow ${BIN}/demoALSA.n $1 $2 $3
