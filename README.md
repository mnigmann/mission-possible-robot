# mission-possible-robot
For a school project, several teams constructed and programmed small robots to navigate 
on uneven terrain and complete various tasks. The required tasks included being able to
drive around, measure the position of the "sun" in the sky, measure the wind speed,
measure the salinity of a puddle of water, measure the temperature of a block of ice,
and plant a flag into the ground. Each team was also required to pick several additional
tasks, according to the number of people in the team. We chose to also collect gravel
from a pile.

The robot was controlled by a Raspberry Pi running the programs in this repository.
For more information on the theory of operation of these programs, see
[https://mnigmann.blogspot.com/2022/10/raspberry-pi-controlled-robot-for.html](
https://mnigmann.blogspot.com/2022/10/raspberry-pi-controlled-robot-for.html)

## Using the program
The program needs the `ncurses` library to work and can be compiled with:
```
gcc robot_main.c http_server.c remote_motor_test.c pwm.c peripherals.c analog_read.c -lncurses -lm -o robot_main -fcommon
```
The program must then be executed as root as it requires low-level access to the Pi's
registers.
