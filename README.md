# Project_Ball_on_Beam_4
Arduino IDE kode for ball balansert på ein bjelke

Hans Christian Finnson
Marien Numbi Mupenda
Hans-Christian Ringstad 
Fredrik Siem Taklo

---------------------------
Construction (outline)

Tuesday 29th Jan. 2019 (?)
- Bought the rail that ball is going to roll down at Biltema, and brought a plank to build the project on. Servo and sensor ordered by Anders Sætermoen
- Cut off a part of the rail to use as the pillar that the rail pivots on
- Cut off pieces of the plank so the pillar and servo could have something to be placed on to keep the rail higher above the ground, allowing for more freedom of movement
- Drilled a hole in the end of the two pieces of rail so they could be attached by a screw and rotate
- Pillar piece had its edges widened so the rail could fit between the edges. 
- Attached the two pieces of rail to eachother

Friday February 1st Feb. 2019
- Received servo and sensors from Anders Sætermoen
- Cut out a small piece of wood for the servo to sit on, and made a 3D-printed frame for the servo to sit in
- Started looking for ways to raise and lower the rail, found a rotateable(?) arm that we decided to use. 
- Started working on a way to attach the arm to the servo. Decided to use a circular plate as a base and then attach a steel/aluminum/whatever beam between it and the arm. 
- Created a mount for the sensors to attach to the end of the rail, and a small barrier at the other end of the rail to prevent the ball from falling off the backside

Tuesday 5th Feb. 2019
- Attached pillar to the plank
- Got breadboard, power cable, wires and potentiometer to attach to the plank
- Tested what position and angle would be best for the servo to raise and lower the rail
- Tried to attach the wooden block from earlier to put servo on, it cracked when getting drilled into the plank, got new block of wood and put servo on that instead. 
- 3D-printed frame proved too weak, and was replaced with a more solid mount
- Drilled out a hole in the bottom of the rail to attach it to the arm. 
- Rail was wobbling a bit from side to side when moving up and down, so decided to put two pillars on each side of the end of the rail to keep it stable when going up and down
- Cut out pillars, found a way to attach it to the plank
- Attached pillars to plank, and removed the small barrier from earlier due to being unnecessary
- Pillars were leaning away from eachother, causing the rail to wobble when high up. Pushed them closer together by connecting the top of the pillars with a piece of metal. 
- Finished attaching arduino components to plank

Thursday 7th Feb. 2019
- Started connecting and adding components to the breadboard and plank.
- Started coding the control system program. 
- Målte opp måleverdier for å gjøre klar til linearisering

Friday 8th Feb. 2019
- Swapped the sensor with another, more accurate sensor due to the readings from the previous one being unreliable
- Added measurement marks to the side of the rail for convenience. 
- Made new readings with the new sensors
- Attached counterweights to the back-end of the rail so the motor didn't have to struggle as much when turning
- Adjusted the position of the arm to help with stability
- Fixed some bugs in the code. Among others, the servo isn't as jumpy anymore and follows the potentiometer more accurately. 

------------

Construction (report)

__ started ___. ___ need a rail to roll the ball on, and ___ 