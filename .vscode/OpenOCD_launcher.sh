#!/bin/zsh
source $HOME/.zshrc

# Function to check if port 3333 is open (listening)
wait_for_openocd() {
    echo "Waiting for openocd to initialize..."
    while ! nc -z localhost 3333; do
        sleep 0.5
    done
    echo "OpenOCD port is open!..."
    # Add an extra delay:
    sleep 1  # Wait an additional 1 second for complete initialization
    echo "OpenOCD is now ready!"
}

# Start the OpenOCD server
start_openocd() {
    echo "Starting OpenOCD server..."
    clear
    openocd -c "set ESP_RTOS none" -c "gdb port 3333" -f /board/esp32s3-builtin.cfg

    # Wait for openocd to be ready
    wait_for_openocd
}

# Stop OpenOCD server
stop_openocd() {
    echo "Killing OpenOCD server..."
    pkill openocd
}

case "$1" in
    start)
        if pgrep -x openocd > /dev/null
        then
            echo "OpenOCD server already running!"
            pgrep -x openocd > /dev/null
        else
            echo "No currently running OpenOCD server instances!"
            start_openocd
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
