<<<<<<< HEAD
## Algorithm

Implemented a wall-following algorithm using ROS2. In the wall follow algorithm the key task is to steer the vehicle to maintain a fixed distance from the wall while navigating curves. The control is maintained and corrected using the **PID (Proportional-Integral-Derivative)** logic.

**The code layout is in form of:**

1. Laser Scan Implementation
2. Error Calculation
3. PID Control
4. Dynamic Speed Control

### Equations used within the Code

1. **Angle of Deviation** 

$$\
\alpha = \arctan\left(\frac{a \cdot \cos(\theta) - b}{a \cdot \sin(\theta)}\right)
\$$
 
2. **Distance to the WALL**

$$\
\text{distance} = b \cdot \cos(\alpha)
\$$

3. **PID Control**


$$\
\text{steering angle} = k_p \cdot \text{error} + k_d \cdot (\text{error} - \text{prev error}) + k_i \cdot \text{accumulated error}
\$$

4. **Error Calculation**

$$\
\text{error} = \text{desired distance} - \text{actual distance} + \text{lookahead distance} * \sin(\alpha)
\$$

$$\
 \text{lookahead distance} = \text{base lookahead} + \text{longitudinal velocity} * \text{lookahead gain}
\$$

5.  **Dynamic Speed Control**

$$\
\text{speed} =
\begin{cases} 
v_{\text{turn}} & \text{if } |\text{steering angle}| > \text{turn threshold} \\
v_{\text{default}} & \text{otherwise}
\end{cases}
\$$
=======
# Asif_F1Tenth_Project
>>>>>>> origin/main
