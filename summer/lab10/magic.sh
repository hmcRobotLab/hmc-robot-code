#!/bin/bash

startme() {
    roscore &
    rosrun ardrone_mudd driver &
    python heliImages.py &
}

stopme() {
    pkill -f "heliImages.py"
    pkill -f "ardrone_mudd"
    pkill -f "roscore"
}

resetme() {
    rosservice call droneControl "reset"
}

case "$1" in
    start)    startme ;;
    stop)     stopme ;;
    restart)  stopme; startme ;;
    reset)    resetme ;;
    *) echo "usage: $0 start|stop|restart|reset" >&2
       exit 1
       ;;

esac

exit
