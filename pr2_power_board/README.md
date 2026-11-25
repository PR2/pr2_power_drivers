## Overview

`power_node2` controls with the PR2 power board. The API below is for
informational purposes only; it is not intended for use by anything
other than [power_monitor](https://index.ros.org/p/power_monitor), which
is where you should look for power data.

The `power_node2` runs on the PR2 and controls the PR2 power board.
The node regulates the main fan speed of the PR2 based on battery and
power board temperature.

## Nodes

### power_node2

#### Subscribed Topics

- `battery/server2`
  ([pr2_msgs/msg/BatteryServer2](https://docs.ros.org/en/rolling/p/pr2_msgs/msg/BatteryServer2.html)):
  Battery data, for temperatures

#### Published Topics

- `/diagnostics`
  ([diagnostics_msgs/msg/DiagnosticArray](https://docs.ros.org/en/rolling/p/diagnostics_msgs/msg/DiagnosticArray.html)):
  Diagnostic information
- `~state`
  ([pr2_msgs/msg/PowerBoardState](https://docs.ros.org/en/rolling/p/pr2_msgs/msg/PowerBoardState.html)):
  Power board state

#### Services

- `~control2`
  ([pr2_power_board/srv/PowerBoardCommand2](https://docs.ros.org/en/rolling/p/pr2_power_board/srv/PowerBoardCommand2.html)):
  Send a command to the power board
- `~control`
  ([pr2_power_board/srv/PowerBoardCommand](https://docs.ros.org/en/rolling/p/pr2_power_board/srv/PowerBoardCommand.html)):
  Send a command to the power board (DEPRECATED)

#### Parameters

- `~sample_frequency` (`float`, default: `10.0`): Sampling frequency
- `~transition_frequency` (`float`, default: `0.1`): Transition
  frequency

### power_node

Deprecated node for controller Rev. A and B power boards.

#### Published Topics

- `/diagnostics`
  ([diagnostics_msgs/msg/DiagnosticArray](https://docs.ros.org/en/rolling/p/diagnostics_msgs/msg/DiagnosticArray.html)):
  Diagnostic information
- `~state`
  ([pr2_msgs/msg/PowerBoardState](https://docs.ros.org/en/rolling/p/pr2_msgs/msg/PowerBoardState.html)):
  Power board state

#### Services

- `~control`
  ([pr2_power_board/srv/PowerBoardCommand](https://docs.ros.org/en/rolling/p/pr2_power_board/srv/PowerBoardCommand.html)):
  Send a command to the power board

## Troubleshooting

1.  format wiki
2.  language en

### Errors when using pr2_power_board

#### Error: Power board "stuck" in standby

##### Is the wireless estop in RunStop?

The display will look like below
![attachment:wirelessestop.png](doc/wirelessestop.png)

**Solution:**

- Is the wireless RunStop (the yellow OMNEX box) in the vacinity of the
  robot? ( In the same room)
- If not find it and bring it into the room, it may be out of range.
- Is the wireless RunStop working and with a yellow light flashing next
  to the start button?
- If not press start
- If the light does not blink after you press start
- Replace the batteries.

##### Is the RunStop button pressed?

The display will look like below.

![attachment:buttonestop.png](doc/buttonestop.png)

**Solution:**

- Is the RunStop button pushed in? (It's the big red button on the
  back.) Twist the button to put in run mode.

#### Error: Power board "stuck" in disable

The display will look like below

![attachment:disabled.png](doc/disabled.png)

**Solution:**

- Try pressing the Reset Disable button for the circuit breaker you want
  on.
- If Reset Disable doesn't work, contact Curt M., Derek K., or Mike S.

#### Error: Power board gui comes up empty

The display will look like below.

![attachment:blankpower.png](doc/blankpower.png)

**Solution:**

- Is the power node running on your robot? Check by running
  

```
rosnode list
```

 and see if pr2_power_board is running.
- If it is not running:
- [Is your ROS_MASTER_URI pointing at the right place?](CommonTroubleshooting#head-284b2d1fa3bf38c9dfedeb10526297ba2963e541)
- Can you run it manually? On the robot 

```
rosrun pr2_power_board power_node
```


- If it comes up you probably have a problem with your launch script(s).
  Check to make sure they launch the pr2_power_board power_node
  correctly. It might look like: 

```
<!-- Power Board Control Node -->
<node pkg="pr2_power_board" type="power_node"  respawn="true"/>
```


- If you cannot run it manually
- Make sure it's compiled and try again 

```
rosmake pr2_power_board
```
