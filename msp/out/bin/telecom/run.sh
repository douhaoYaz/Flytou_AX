#!/bin/sh
cur_path=$(cd "$(dirname $0)";pwd)

process=telecom

pid=$(pidof ${process})
if [ $pid ]; then
  echo "${process} is already running, please check the process(pid: $pid) first."
  exit 1;
fi

if [ -e /tmp/core* ]; then
  echo "exist coredump file under path: /tmp, please deal with coredump file first."
  exit 1;
fi

# 0: None; 1: Critical; 2: Error; 3: Info; 4: Debug
export AX_OPAL_LOG_LEVEL=2

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./lib

# 0: OS04A10 1: IMX334 2: GC4653 3: OS08A20
export AX_OPAL_SENSOR_ID_SETTINGS=2
export AX_OPAL_OSD_FONT_FILE=./res/GB2312.ttf
export AX_OPAL_OSD_ONLINE_ENABLE=1
export AX_OPAL_OSD_OFFLINE_ENABLE=0

# AX620 demo use ES8388 codec
# 0: ES8311 (default)
# 1: ES8388
export AUDIO_CODEC_CHIP=1

# disable kernel log
echo 0 > /proc/sys/kernel/printk

./${process} $*
