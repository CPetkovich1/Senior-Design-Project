# Senior-Design-Project
For startup make sure to run the command on the robot: fastdds discovery -i 0 -l 192.168.4.1 -p 11811 or fastdds discovery --server-id 0 --udp-address 192.168.4.1 --udp-port 11811
This starts the server for wireless ros communication between control station and the robot

Also may need to run these commands: pkill -9 _ros2_daemon
                                     ros2 daemon start
in order to reset daemon on the control station in order to see topics and nodes for wireless communication.
