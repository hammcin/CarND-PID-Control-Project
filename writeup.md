# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Goals

Build a PID controller and tune the PID hyperparameters.  Describe the effect each of the P, I, D components had.  Describe how the final hyperparameters were chosen.

Test your solution on the simulator.  The vehicle must be able to successfully drive a lap around the track.  No tire may leave the drivable portion of the track surface.  The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

## PID Controller

The PID controller is implemented in the PID class (PID.h, PID.cpp).  The P and I errors are each initialized to 0.0 (PID.cpp, PID::Init, lines 11-20).  On each iteration of the controller, the P, I, and D errors are updated using the cross track error (PID.cpp, PID::UpdateError, lines 22-29) and the steer value is calculated to be proportional to the P, I, D errors using the P, I, D coefficients as proportionality constants (PID.cpp, PID::TotalError, lines 31-38).

## PID Hyperparameters

The initial hyperparameters (Kp: 0.2, Ki: 0.004, Kd: 3.0) were chosen using manual tuning.  Using the initial hyperparameter values, the twiddle algorithm was then used to perform parameter optimization.  For the twiddle algorithm, dp was chosen to have the same order of magnitude as the initial hyperparameter values (Kp: 0.1, Ki: 0.001, Kd: 1.0).  To begin the twiddle procedure, an initial value for mean squared error was needed (main.cpp, lines 102-133).  To calculate the error for twiddle, the PID controller ran for n=100 iterations using a given set of hyperparameters and the mean squared error of the cross track error was taken as the error for those iterations.  Given an initial error measurement, twiddle then proceeded to perturb the hyperparameter values and test whether the perturbation improved the mean squared error measured, as before, using the cross track error over n=100 iterations (main.cpp, lines 138-284).  The twiddle algorithm ran until the vehicle was able to successfully drive a lap around the track (Kp: 0.18, Ki: 0.004, Kd: 3.0).

The P component of the controller adjusted the steer value based on the cross track error.  The P component has a tendency to overshoot the goal trajectory, resulting in an oscillatory trajectory around the goal trajectory.  This behavior was observed in the simulator for Kp=0.18 (P_0.18 Control Project 2019-07-08_15-34-26.mp4).

The D component of the controller adjusted the steer value based on the difference between the current cross track error and the cross track error from the previous time step.  The D component damps the overshoot of the P component over time, so that the oscillatory behavior of the P component dies out and the vehicle trajectory converges to the goal trajectory.  This behavior was observed in the simulator for Kp=0.18 and Kd=3.0 (P_0.18 D_3.0 Control Project 2019-07-08_16-06-07.mp4).  After adding the D component to the controller, the vehicle was able to successfully drive a lap around the track.

The I component of the controller adjusted the steer value based on the sum of all the cross track errors that the controller has observed so far.  The I component compensates for any bias present in the steering.  This behavior was not observable for Kp=0.18, Kd=3.0, and Ki=0.004 (PID Control Project 2019-07-08_15-20-50.mp4).  The reason for this could be that the bias in steering was very small or not present at all.
