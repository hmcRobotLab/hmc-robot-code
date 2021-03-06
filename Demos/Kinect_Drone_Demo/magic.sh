#!/bin/bash

startme() {
    roscore &
    rosrun openni_camera openni_node &
    rosrun pydrone pydrone.py &
}

stopme() {
    pkill -f "pydrone.py"
    pkill -f "openni_node"
    pkill -f "roscore"
}

case "$1" in
    start)    startme ;;
    stop)     stopme ;;
    restart)  stopme; startme ;;
    *) echo "usage: $0 start|stop|restart" >&2
       exit 1
       ;;

esac

exit