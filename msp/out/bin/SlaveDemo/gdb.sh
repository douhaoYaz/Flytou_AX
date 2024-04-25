#!/bin/sh
cur_path=$(cd "$(dirname $0)";pwd)

process=slave_demo

pid=$(pidof ${process})
if [ $pid ]; then
  echo "${process} is already running, please check the process(pid: $pid) first."
  exit 1;
fi

if [ -e /tmp/core* ]; then
  echo "exist coredump file under path: /tmp, please deal with coredump file first."
  exit 1;
fi

if [ $# == 0 ] ; then
  json_path=$cur_path/config/cascade_8_4_hdr.json
elif [ $# == 1 ] ; then
  json_path=$1
else
  echo "USAGE: $0 <configure file path>"
  echo " e.g.: $0 ./config/cascade_8_4_hdr.json"
  exit 1;
fi

if [ ! -f $json_path ]; then
  echo "ERROR: Config file \"$json_path\" not found!"
  exit 1;
fi

#set -e
cd $cur_path

# load config
source ./config/${process}.conf

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./lib

# Open core dump
if [ $EnableCoreDump == 1 ] ; then
  ulimit -c unlimited
  echo /var/log/core-%e-%p-%t > /proc/sys/kernel/core_pattern
fi

# launch
if [ $RunInBackground == 1 ] ; then
  # Always export log to file if run in background
  nohup gdb --args ./${process} $json_path 2 $LogLevel $ChnDbgLevel $PrintFPS $TestMode $GetYuvFrmTimeout $EnableIVPS &
else
  gdb --args ./${process} $json_path $LogTarget $LogLevel $ChnDbgLevel $PrintFPS $TestMode $GetYuvFrmTimeout $EnableIVPS
fi
