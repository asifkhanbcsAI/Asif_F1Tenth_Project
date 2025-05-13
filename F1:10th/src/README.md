# Wall Follow

**Wall-following algorithm for Autonomous Navigation**

This algorithm enables a robot or vehicle to maintain a constant distance from a wall using range sensor data (e.g., lidar, sonar, ir). it uses control logic—typically a pid controller—to continuously correct the heading based on the error between the desired and measured distances from the wall.

---

### How it Works

1. **sensor reading:**  
   measure the current distance to the wall using side-facing sensors.  
   let:  
   - $$\ d_{\text{desired}} $$ = desired distance to wall  
   - $$\ d_{\text{actual}} $$ = current distance to wall  

2. **Calculate Error:**  

$$
e(t) = d_{\text{desired}} - d_{\text{actual}}
$$

3. **PID Control Law:**  
   the control output \( u(t) \), used to adjust steering, is computed as:  

$$
u(t) = K_p \cdot e(t) + K_i \cdot \int_0^t e(\tau) \, d\tau + K_d \cdot \frac{de(t)}{dt}
$$


   where:  
   - $$\( K_p \), \( K_i \), \( K_d \)$$ are the proportional, integral, and derivative gains respectively

5. **Apply Control:**
- adjust the vehicle’s heading angle or steering rate to minimize $$\ e(t) \$$

---

This algorithm is effective in:
- Exploring unknown environments  
- Navigating narrow corridors  
- Following curved or straight boundaries without collision
