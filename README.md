# Obstacle-Avoiding Rover

A small 2WD rover that measures the distance to obstacles with an ultrasonic sensor and drives around them on its own. It is built for the LUTES Deep Tech Hackathon 2026, challenge "Small Robot That Measures Distance to Obstacles".

The robot can be started, stopped and driven by hand from a web dashboard. The dashboard shows a radar view of the sensor, a map of the path and the detected obstacles, and a distance graph.

## Hardware

- Arduino UNO R4 WiFi
- 2 x 28BYJ-48 stepper motors with ULN2003 driver boards (drive wheels)
- HC-SR04 ultrasonic sensor on an SG-90 servo (scans 60 to 120 degrees)
- 2 x 18650 cells (7.4 V) with an on/off switch
- Chassis plate, 2 wheels (6.8 cm) and a rear caster

## Wiring

| From | To |
|---|---|
| Battery + (through switch) | VIN |
| Battery - | GND |
| Arduino 5V / GND | Breadboard rails |
| Left ULN2003 IN1, IN2, IN3, IN4 | D2, D3, D4, D5 |
| Right ULN2003 IN1, IN2, IN3, IN4 | D6, D7, D8, D9 |
| Both ULN2003 +/- | 5V / GND rails |
| SG-90 signal / red / brown | D10 / 5V / GND |
| HC-SR04 Trig / Echo | D11 / D12 |
| HC-SR04 VCC / GND | 5V / GND |

## How it works

- The steppers are driven from a 10 kHz timer interrupt, so measuring distance and serving the web page never interrupt the stepping.
- While driving, the robot measures the distance ahead every 60 ms. Below 20 cm (confirmed by a second reading) it stops, reverses 15 cm and scans 7 directions between 60 and 120 degrees.
- It then drives a 45 degree curve with a 25 cm radius towards the better side. If both sides are blocked it reverses along a curve instead.
- Position and heading are calculated from the step counts. The robot keeps a 10 cm grid of the places it has already visited and avoids turning back into them.
- Because the steppers give no feedback, the robot detects being stuck by comparing the distance the wheels should have moved with the change in the measured distance ahead.

## Calibration

| Value | Result |
|---|---|
| Wheel diameter | 6.8 cm (30 cm commanded drove 34 cm with 6.0) |
| Track width | 21.6 cm (360 degree spin test) |
| Drive speed | 1100 half-steps/s (about 5.7 cm/s); 1500 skips steps |
| Servo range | 60 to 120 degrees, 90 is straight ahead |

## Dashboard

The robot joins the laptop hotspot with the static address 192.168.137.50 and serves the dashboard at `http://192.168.137.50`.

- START / STOP: automatic driving
- Manual control: hold the arrow buttons or the arrow keys
- Speed sliders for driving and turning
- Radar, map, distance graph, statistics and an event log

## Building

1. Install the Arduino IDE and the "Arduino UNO R4 Boards" package.
2. Install the libraries AccelStepper and Servo.
3. Set the WiFi name and password at the top of `RoverAvoid/RoverAvoid.ino`.
4. Open `RoverAvoid/RoverAvoid.ino` and upload it to the board.
