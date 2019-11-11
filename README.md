# PID-Controller
Proportional Integral Derivative algorithm to smoothly control a car's direction and velocity through steering angle and throttle actuators.

![PIDController](images/PIDController.gif)

Let's say you are driving on the right hand side of your lane and you want to get centered in the middle. How do you steer? Do you just steer left until you get to the middle and then stop steering? 

Do the test! You'll become aware that you have a PID controller embedded in your instincts. At the beginning, you steer harder to the left to get your car going towards the middle of the lane. Before you get there, you steer right to align the car to the road again. In conclusion, you steer both left and right for this maneuver. 

[PID](https://en.wikipedia.org/wiki/PID_controller) is one of the most popular control algorithms. It's a hundred years old and is still used because it gets the job done.

In this project I implemented two PID controllers to drive the car around the track.
The project is implemented in C++ and the source code can be found in the `src` folder above. The PID class is implemented under `PID.h` and `PID.cpp`. The steering angle and throttle values are calculated under `h.onMessage` function in `main.cpp`.

The starter code for this project is provided by Udacity and can be found [here](https://github.com/udacity/CarND-PID-Control-Project).

## What Is PID?

PID is a closed loop feedback controller. It produces a control value based on the error between the current state of the system and the desired state.

![PIDeq](images/PIDeq.JPG) 

In the equation above `u(t)` is the control value and `e(t)` is the error. Let's say the car's desired position is in the middle of the lane and its current position is 2m to the right. The error in this case is 2m. 

`Kp` is the proportional gain. Using `Kp` the car will steer left while moving forward down the road. The steering value will be large at the begining as the error is large. As the car approaches the desired centered position, the steering angle approaches zero. When the car is in the middle of the road the steering angle is zero. Even so, the car will keep going left since it is still slightly facing that way.
The `Kp` component of the controller helps steer the car in the correct direction. Using `Kp` alone in an inertial system like a car leads to oversteer. With this, the car will drive in an S shape along the desired trajectory.

`Kd` is the derivative gain. It is used together with `Kp` to compensate for the overshoot. `de(t)/dt` is the derivate of the error. As the error gets smaller in time, the derivative is negative. The faster the car is approaching the center of the road, the stronger the derivative effect. When the error gets smaller, `Kd` will overpower `Kp` effect and the car will start steering in the opposite direction to get alligned with the road. 

`Ki` is the integral gain. In a discrete system like an embedded ECU, this is calculated as the sum of the error over time. The integral is used if a control value is needed to keep the system in the desired state; i.e.: when the error is zero. For a well calibrated direction system in a car, the integral component is not needed when controlling the car's trajectory through the steering angle. On a straight line the car should be able to drive holding its steering angle at zero. 

To control the velocity on the other hand, the integral is needed because we still need the throttle value to be non-zero to keep going at 30MPH, even if the car's speed is already 30MPH. 


## PID Implementation
The PID controller is implemented using the PID class. 
Total error is used to calculate the control value in a similar way as the equation from above.

```
double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double total_error;
  total_error = Kp * p_error + Kd * d_error + Ki * i_error;
  //std::cout<<"The composition of steering angle is: "<<std::endl;
  //std::cout<<"Kp "<< Kp <<" * p_error "<<p_error<<" + Kd "<< Kd <<" * d_error " << d_error <<"= "<<total_error<<std::endl;
  return total_error;  // TODO: Add your total error calc here!
}
```

Printing out the decomposition of the control value is useful for debugging when choosing the right `Kp`, `Kd` and `Ki` values.

The `p_error`, `i_error` and `d_error` are calculated below. The `p_error` is the straight value taken from the feedback loop. It represents the difference between the desired system state and its current state. `d_error`, or the derivate, is simply the substraction between the previous error and the current one.  `i_error`, or the integral, is the sum of errors over time.

```
void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  i_error += cte;
  d_error = cte - p_error; //it can be initialized with CTE value because the simulator is responsive only after 2 cycles
  p_error = cte;
}
```

## Controlling The Direction
The first controller in this project is the one that, through the steering angle, makes the car drive along the road as close as possible to its center.

At the beginning, I set the steering angle to a hardcoded value of zero and launched the simulator to see if the car goes straight. I noticed that the steering angle was correctly set to zero, with no deviation, and the car seemed (visually) to be driving straight. This is how I decided that, for the direction control, the integral component is not needed. I chose a PD controller.

For the `Kp` and `Kd` values, one needs to know the dynamics of the process to be able to calculate them. The car is approximated to a bycicle and the equations are already given in the Udacity class.

```
        # Execute motion
        turn = np.tan(steering) * distance / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance * np.cos(self.orientation)
            self.y += distance * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)
```

I changed the vehicle's length to 4 meters and used the motion model in the class to tune the `Kp` and `Kd` paramaters.

![PD4m](images/PD4m.JPG) 

`Kp = 0.5` and `Kd = 2.5` seem to provide a good vehicle trajectory in following a reference line.

![PD0525worksonplotunstabledriving](images/PD0525worksonplotunstabledriving.JPG)

I plugged these values into the implementation and started the simulator. The vehicle was very unstable, the oscilations were strong, and the car was not even able to drive on a straight line.

This behavior was very surprising since the results looked good on the graph. Then, I realized that the car was steering with an angle in between -25deg and +25deg when the control values were given between -1rad and 1rad corresponding to -57deg and +57deg.

Then I monitored the control values and the actuator values in debug to see what is happening.

![Factor](images/Factor.JPG)

And here is the problem. The car does not steer with the controller provided value and it has a 2/3 reducing factor.

I integrated this factor in the vehicle's dymanic model and confirmed that this is the source of the unexpected oscillations.

![PD0525fact23](images/PD0525fact23.JPG)

With this updated vehicle model, I chose to increase the `Kd` value to compensate the overshooting.
I chose `Kp = 0.5` as before and increased `Kd = 3.5`.

![PD0535fact23straightokunstableincurves](images/PD0535fact23straightokunstableincurves.JPG)

I got the simulator running and this time the car was able to follow a straight line trajectory. That's a great result!

The remaining problem is that it becomes very unstable in curves. I took a look at how the control value is calculated and realized that such a big `Kd` induces the instability. When driving into a curve, the error grows very fast making the `Kd` component built a lot on top of `Kp` and results in oversteering.

I then used the vehicle model to find a `Kp` `Kd` combination that doesn't overshoot, but allows for `Kd` to be smaller.

I chose `Kp = 0.04` and lowered `Kd = 1`.

![PD0041straightslowoncurves](images/PD0041straightslowoncurves.JPG)

In the simulator the vehicle is still able to drive on a straight line, but it is too slow to turn into curves and the car gets off the track.

I kept tuning the parameters from this point by trial and error. I got a pair `Kp = 0.1` and `Kd = 1.2` that seemed like a good compromise. These are my final values.


## Controlling The Velocity

For the velocity controller the values were tuned manually since I don't have the car's velocity dynamics equations.  Choosing a hardcoded value of 0.3 for the throttle gets the vehicle running at 37 MPH in a pretty constant manner. The only thing to notice is that the vehicle takes some time to get up the speed. 

I decided to use a PI controller. Here the integral component is obviously useful. When the error is zero, the throttle value is maintained to keep the car moving at the desired velocity. I could have included the Kd component as well. It could serve in case the error decreases too fast. Then, the Kd would reduce the throttle anticipating that the car will go over the set velocity.

I set the desired speed at 30MPH and chose `Kp = 0.01` and `Ki = 0.001` for my first trial. With these values the simulated car starts from 0MPH, increases speed, gets up to 50MPH and then slows down and stabilizes at 30MPH.

Even without a graph to look at, it is obvious that the `Ki` factor is too large. The error gets intergated over a long period of time. I didn't want to increase the `Kp` factor to make the initial acceleration faster since: `0.01(Kp) * 30(initial error) = 0.3` (the approximate throttle to drive at desired speed). So increasing `Kp` would lead to instability in reaching the desired velocity.

With `Ki = 0.001` the integrated error factor is too large. When the error reaches zero, the `Ki * sum(e(t))` is larger than 0.3, the approximated desired throttle. Because of this the car accelerates and increases the velocity up to 50MPH. During this time the error is negative and keeps being intergated. `Kp` component slows the vehicle down and when the 30MPH is reached again, the `sum(e(t))` is smaller than it was before, and manages to provide the desired throttle to maintain the speed.

I kept `Kp = 0.01` and lowered the integral factor ten times to `Ki = 0.0001`.
These are my final values.


## Simulation Video


[![HighwayDriving](https://img.youtube.com/vi/YQ1eC_hYavI/0.jpg)](https://www.youtube.com/watch?v=YQ1eC_hYavI)

 Click on the image to see the video!

 To download the simulator go [here](https://github.com/udacity/self-driving-car-sim/releases).