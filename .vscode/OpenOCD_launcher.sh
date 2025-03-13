#!/bin/zsh

# Function to check if the OpenOCD server is already running
is_openocd_running() {
    pgrep openocd > /dev/null
}

# Start the OpenOCD server
start_openocd() {
    echo "Starting OpenOCD server..."
    clear
    openocd -c "set ESP_RTOS none" -f /board/esp32s3-builtin.cfg
}

case "$1" in
    start)
        if !is_openocd_running; then
            start_openocd;
        fi
        ;;
    stop)
        stop_openocd
        ;;
    *)
        echo "Usage: $0 {start|stop}"
        exit 1
        ;;
esac
