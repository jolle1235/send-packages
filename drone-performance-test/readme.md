Drone performance evaluation script by QuadSAT.

This script is intended to be run on a companion computer connected to the drone to be tested.

How to use:
1. Install requirements by running pip install -r requirements.txt
2. Connect companion computer to the flight controller through UART
3. When ready, run the flight_ctrl.py program through an SSH connection, passing the port and baud rate of the flight controller serial connection. 
    Note that the connection either needs to be kept throughout the flight, or the program needs to be run in the background by running python flight_ctrl.py PORT BAUD &
4. Arm the drone and fly it to an adequate altitude (10-20m or so), and switch it to guided mode.
5. Wait for the drone to finish the predetermined path. 
6. There is a single point position at the end of the flight, where the drone stays for a few minutes
7. Once it is done, the drone will fly back to 20m over the takeoff point.
8. Switch back to manual mode, and land the drone (or use RTL).
9. Download the data from the "data" folder, and send it to QuadSAT for investigation, together with the drone log file from the flight.


About the flight:
Once in guided mode, the flight_ctrl.py will take over and fly the pre-defined flight path by sending position commands.

The flight stays below 100m altitude over the takeoff position.

The flight takes around 15 minutes.

The flight takes place in the north-west and north-east quadrants from the takeoff point.

The flight path can be seen in the screenshot Flightpath.png (ignore the waypoints).

The flight consists of the following steps:
1. Wait for arm and takeoff.
2. Fly to safety height (20m) over the takeoff position.
3. Fly the start of the first half-circle, 80m east of the takeoff position, at 50m altitude.
4. A counter-clockwise, horizontal half-circle with 80m radius at 1.5 degrees/s
5. A clockwise, horizontal half-circle with 80m radius at 4 degrees/s
6. A vertical half-circle overhead of the takeoff position, at 80m radius, at 1.5 degrees/s
7. A vertical half-circle overhead of the takeoff position, at 80m radius, at 2 degrees/s
8. A few horizonal straight lines at altitudes between 40 and 60m, at speeds between 3 and 8m/s
9. A single position, 50m above the takeoff position, for 3 minutes.

Once these steps are done, the drone flies down to 20m over the takeoff positon and waits for the pilot to take over and land.