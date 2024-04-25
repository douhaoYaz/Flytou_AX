#!/bin/sh
cur_path=$(cd "$(dirname $0)";pwd)

process=sample_opal

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

# if use AX620A demo, pls enable below
# export AXERA_DEMO_ES8388=1

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

if [ $# = 0 ]; then
    ./sample_opal video
elif [ "$1" = "video" ]; then
    ./sample_opal video
elif [ "$1" = "audio" ]; then
    if  [ "$2" = "-t" ]; then
         # audio testflow
        ./sample_opal audio -t 1 -b 16 -r 8000 -e g711a
    elif [ "$2" = "-c" ]; then
         # audio capture
        ./sample_opal audio -c 1 -b 16 -r 8000 -e g711a
    elif [ "$2" = "-p" ]; then
        # adido play
        ./sample_opal audio -p 1 -b 16 -r 8000 -e g711a
    fi
elif [ "$1" = "-h" ]; then
    echo "usage: "
    echo "./run.sh               #run video sample default"
    echo "./run.sh video         #run video sample"
    echo "./run.sh audio -t      #run audio testflow 30s"
    echo "./run.sh audio -c      #run audio in capture sample"
    echo "./run.sh audio -p      #run audio out play sample"
fi
