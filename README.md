# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program


## My Solution Write Up

For this solution, I used the provided code, as well as some of the class exercises code to jump start it. I was very quickly able to get the Model Predictive Controller running at high speeds and completing the track without having the tire go off the track. However, for my solution work with latency I had to make some changes. I tried different approaches before realized the solution was simpler that I first expected: I use the fitted polynomial to make most predictions about the state after 100ms latency.

### Model

The first step of my model was to fit a third degree polynomial on the next 6 waypoints provided by the simulator. To facilitate, the waypoints were transformed from the global positioning of the track to the car coordinate position. The car coordinate position has the car moving forward along the x axis, while any turning would cause disaplacement along the y axis.

Next step of the model was to create a state of the vehicle so the model could optimize the actuators over the next steps to best fit along the polynomial. The state included the following values:

- x (car relative coordinates)
- y (car relative coordinates)
- psi (turning angle)
- velocity
- cross check error
- turning angle error

The goal of the model was to optimize the following actuators to best fit the waypoints:

- Throttle
- Steering Angle


#### Handling Latency

In order to properly handle the 100ms, I couldn't use the current state of the vehicle, as the actuators would change by the time it should already be at the next step, considering the 100ms step interval. In order to properly handle the car, I had to estimate the state 100ms in the future. Here's how I estimated:

First step was to predict the velocity, as the rest of the state values would depend on it:

```
Predicted_Velocity = Velocity + (Throttle * Latency)
```

After that, I had to predict the angle the car would be, which would dependon the rate at which the car was turning:

```
Predicted_Angle = Predicted_Velocity * Latency * Steering_Rate / 2
```
The division by 2 was to average the rate of change, as the Steering_Rate would have changed over time during the 100ms interval.

Using the estimated velocity and angle, estimate the x and y position:

```
# How much the car moved in the x and y axis
X_Displacement = Predicted_Velocity * Latency * cos(Predicted_Angle)
Y_Displacement = Predicted_Velocity * Latency * sin(Predicted_Angle)

# Estimate in the track global position
Global_X = Previous_X + X_Displacement * cos(Predicted_Angle) - sin(Predicted_Angle)
Global_Y = Previous_X + X_Displacement * sin(Predicted_Angle) + cos(Predicted_Angle)
```

Transforming the Global_X and Global_Y into the car coodinate space would give an x,y coordinate value in the fitted polynomial that would represent the estimated position of the car 100ms in the future.

By assuming that our polynomial is fitted to the center of the track, we can also estimate the cross track error by evaluating the x position of the vehicle and getting the expected y along the polynomial, and substracting the estimated y position. That is not the true cross track error, but works well enough for the problem.

The angle error can be estimated with the following formula:

```
Predicted_Angle_error = Predicted_Angle - atan(coeffs[1] + 2 * Predicted_X * coeffs[2] + 3 * coeffs[3] * pow(Predicted_X, 2))
```
where coeffs is the vector of coefficients of the fitted polynomial.


### Number of Steps and Steps Interval

Since the simulated latency was set to 100ms (which is the expected actuator latency on a real world scenario), I set the interval between steps to that same 100ms. Making the steps shorter than that would not bring benefits as the actuators wouldn't respond on a smaller interval than that. There was also no reason to increase the step interval to over 100ms.

For the number of steps, I experimentes with extreme short numbers and long numbers. Having a small number of steps (around 5) would case the polynomial to have unnecessary sharp curves, making the car lose control and having unnatural turning. Having a bigger number of steps (around 20) would work well for a lot of the steps, with the exception of when the cost function would rise, usually around curvers, which would cause the polynomial to fit weird shapes.

I settled on 12 steps, which didn't show any behavior that the shorter or bigger number of steps had.
