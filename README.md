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


```
Lf = 2.67 # Length from front to CoG
v[t+1] = v[t] + Throttle * Latency
psi[t+1] = Latency * v[t] * Steering_Rate / Lf

# How much the car moved in the x and y axis
x[t+1] = v[t] * Latency * cos(Psi[t+1])
t[t+1] = v[t] * Latency * sin(Psi[t+1])

cte[t+1] = abs(y[t+1] - f(x[t+1]))
epsi[t+1] = psi[t+1] - atan(coefficient[1] + 2 * x[t+1] * coefficient[2] + 3 * coefficient[3] * x[t+1]^2);
```
where f() is the fitted polynomial and coefficient[n] is the coefficient of the polynomial at the order n.



### Number of Steps and Steps Interval

Since the simulated latency was set to 100ms (which is the expected actuator latency on a real world scenario), I set the interval between steps to that same 100ms. Making the steps shorter than that would not bring benefits as the actuators wouldn't respond on a smaller interval than that. There was also no reason to increase the step interval to over 100ms.

For the number of steps, I experimentes with extreme short numbers and long numbers. Having a small number of steps (around 5) would case the polynomial to have unnecessary sharp curves, making the car lose control and having unnatural turning. Having a bigger number of steps (around 20) would work well for a lot of the steps, with the exception of when the cost function would rise, usually around curvers, which would cause the polynomial to fit weird shapes.

I settled on 12 steps, which didn't show any behavior that the shorter or bigger number of steps had.
