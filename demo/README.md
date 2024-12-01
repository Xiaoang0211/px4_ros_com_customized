# Drone Exploration and Collision Prevention Demo

This page demonstrates the random exploration and collision prevention strategies for effective autonomous navigation and mapping.

---

## Random Exploration Demo

The drone employs three types of random exploration policies for exploration new environment:

1. **Forward/Backward Movement:**  
   The drone moves forward or backward, aligned with its current yaw angle, with a velocity magnitude uniformly sampled between **0.5 m/s** and **1.5 m/s**.

2. **Position Locked Mode:**  
   While maintaining a fixed position, the drone rotates within a yaw range of **±37.5°** to scan its surroundings.

3. **Horizontal Movement:**  
   The drone moves horizontally in a random direction with a randomly sampled velocity magnitude.

![Random Exploration Demo](./demo_random_explore.gif)

---

## Collision Prevention Demo

Two types of **Collision Prevention** policies are implemented to handle concave and non-concave spaces efficiently for mapping and exploration:

1. **Concave Space Handling:**  
   - The drone performs a **180° turn**, aligning its camera's yaw angle with the new velocity direction to escape the concave area.

   ![Concave Space Demo](./concave_demo.gif)

2. **Non-Concave Space Handling:**  
   - The drone's velocity vector is adjusted to be **perpendicular to the closest obstacle**.  
   - In cases where the drone is flying towards a wall, the velocity will be adjusted to run **parallel to the wall**, allowing smooth navigation along the obstacle.
   - The camera's yaw angle is aligned opposite to the direction of the closest obstacle, enhancing environmental awareness.

   ![Non-Concave Space Demo](./nonconcave_demo.gif)

---

