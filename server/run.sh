#!/usr/bin/env bash
# Run the LED server in the background so it survives closing the terminal/SSH session.
#
#   ./run.sh [start]   build and start (default)
#   ./run.sh stop      stop the running server
#   ./run.sh restart   stop, rebuild and start
#   ./run.sh status    show whether it is running
#   ./run.sh log       follow the log (Ctrl+C only stops following, not the server)
#
# The port comes from PORT (default 80), e.g. `PORT=8080 ./run.sh`.
set -euo pipefail

cd "$(dirname "$0")"

PORT="${PORT:-80}"
BIN=target/release/server
PID_FILE=server.pid
LOG_FILE=server.log

running_pid() {
    [[ -f $PID_FILE ]] || return 1
    local pid
    pid=$(<"$PID_FILE")
    if kill -0 "$pid" 2>/dev/null; then
        echo "$pid"
    else
        rm -f "$PID_FILE"
        return 1
    fi
}

start() {
    if pid=$(running_pid); then
        echo "already running (pid $pid), use './run.sh restart' to restart"
        return
    fi

    cargo build --release

    # Ports below 1024 need a capability. Rebuilding replaces the binary and drops it, so check every time.
    if (( PORT < 1024 )) && ! getcap "$BIN" | grep -q cap_net_bind_service; then
        echo "port $PORT needs cap_net_bind_service, granting it (asks for your password)"
        sudo setcap cap_net_bind_service=+ep "$BIN"
    fi

    # setsid + nohup detach the server from this terminal, so it keeps running after logout.
    PORT="$PORT" NO_COLOR=1 setsid nohup "$BIN" >>"$LOG_FILE" 2>&1 </dev/null &
    echo $! >"$PID_FILE"

    sleep 1
    if pid=$(running_pid); then
        echo "started (pid $pid) on port $PORT, logging to $(pwd)/$LOG_FILE"
    else
        echo "server exited right away, last log lines:"
        tail -n 20 "$LOG_FILE"
        exit 1
    fi
}

stop() {
    if ! pid=$(running_pid); then
        echo "not running"
        return
    fi
    kill "$pid"
    for _ in {1..50}; do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.1
    done
    rm -f "$PID_FILE"
    echo "stopped (pid $pid)"
}

case "${1:-start}" in
    start) start ;;
    stop) stop ;;
    restart) stop; start ;;
    status)
        if pid=$(running_pid); then echo "running (pid $pid)"; else echo "not running"; fi ;;
    log) tail -n 50 -f "$LOG_FILE" ;;
    *) echo "usage: $0 [start|stop|restart|status|log]"; exit 1 ;;
esac
