README.txt

Install: Simply download the code from our git repo at: https://github.com/brianc217/cs205_final

Files: There are four main files from which you can run code:
	1. serial_bare.py - Implements multiple robot simulation in serial without plotting and minimal data output. The goal was to only report the fastest robot's time and their starting velocity.
	2. par_bare.py - The parallel version of serial_bare using MPI
	3. serial.py - Implements similar multiple robot simulation, except takes robot-to-robot interactions into account. It also reports much more information at the end of the run, including graphical data.
	4. griddedParallel - Implements parallel version of serial.py using MPI and by splitting up the work between processes by gridding up the map into geographical regions. We didn't pursue this extensively because we found early on that communication was causing too much overhead.

Parameters:
	There are several variables that you can play around with for interesting results. Generally, the files have the same variables you can change as follows and they are located at the top of the main function in each file:
		1. startPos - The x,y coordinate of the starting position of all of the robots, should not be located inside rigid obstacles
		2. maxNumRobots - (N.B. - this is called totalRobots in par_bare.py) The maximum number of robots to deploy to the field. A new robot will be instantiated every 5 time steps until time expires
		3. goalPosition - The x,y coordinate for the goal destination for the robots
		4. You can also play with the rigid obstacle location (rigidPos) and other variables as you see fit.
		5. The number of processes used for griddedParallel.py should remain at 4 but the number is flexible for par_bare.py

How To Run:
	python serial.py
	python serial_bare.py
	mpiexec -n 16 python par_bare.py
	mpiexec -n 4 python griddedParallel.py