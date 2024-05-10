#!/bin/bash

# Get a list of all processes with 'ros' in their name
processes=$(pgrep -f ros)

for pid in $processes
do
    # Get the process name
    process_name=$(ps -p $pid -o comm=)

    # Get the process command
    process_command=$(ps -p $pid -o args=)

    # If the process name is not 'rqt' or 'rqt_graph', kill it
    if [[ "$process_name" != "rqt" && 
          "$process_command" != *"ros2-daemon"* &&
          "$process_name" != "rqt_graph" ]]; then
        echo "Killing $process_name with PID: $pid"
        kill -SIGTERM $pid
    fi
done
