% Progress Update
% Saputra V.B.
% Nov 03, 2015

# Overview

- Radar Development
- Simulation Development as platform to test perception and localization on COMS


# Radar Development
<br>
<br>
*Code is complete and Testing now*

  - There isn't any cable prob on esr final, but the data also not good
  - Need to keep in mind that canbus need 120 ohms termination
  - Will keep testing and collecting data
  - test on road, and while moving, with egomotion
  - the esr has maximum yaw angle, this would imply the need to change mounting angle
  - there is an error to be investigated, but lack of documentation

# Radar Development
**Integration**

  - Susu's filtering, if he is around
  - Integration with PanYu's, using Sensor Fusion

**Mitigation**

  - hardware solution: may need to replace all with sms

# Radar Development
**Conclusion**

  - Draft Alphard, to replace all radar with sms, integrate egomotion with ZW
  - Final Alphard, continue testing with the main team for integration and recover Delphi (if possible)

# Radar Interface
![Radar Control](images/Radar Control_002.png)

# Radar Interface
![Radar Status GUI](images/Radar Status center_front_esr_001.png)

# Radar Interface
**Generic Template**

  - During development, created code for processing all canbus related devices
  - All process that related to canbus message processing, can reuse the template
  - Configurable using Yaml
  - Currently support rosbag and rviz of ROS, but they are decoupled to integrate with others


# Simulation
**As Development Platform**

  - Using V-REP (free, or can buy license if used for commercial purposes)
  - To test and choke algorithm stability without driver and other hardware specifics
  - Mitigate bugs and human errors early before real tests
  - Ideal for testing algorithm as data are without noises, and later we can compare with real data

# Simulation
**As Testing Tool**

  - Good for multiple robot, to verify interactions
  - Quickly prototype algorithm for new environment
  - hardware abstraction, decouple development, flip flag to switch
  - identify problem early, especially those related to fine tuning the algorithm
  - Determine best sensor mounting
  - Visualization
</section>

# Simulation Model
**Sensors**

- IMU/DMI -> odometry
- Lidar
- Camera
- Safety proximity triggered sensor
- Motor mounted sensors

# Simulation Model
**Actuators**

- Motors
- Steering
- Vehicle body

# Simulation Model
**Processes**

- Sensor filters
- Control
  - Path planning
- Perception (localization and vision)

# Simulation Model
  **Environment**

  - Map Objects
  - Static object
  - Dynamic Object
- Various Scenarios (combination between objects)

# Simulation Model
  **Realism**

  - Noise distribution
  - Sensor malfunction, driver died
  - actuator choke
  - collision
  - add anomalies on the map

# Simulation Early Results

# VRep Problems
![Velo Problem](images/VREP_PROBLEM.png)
</section>
<section>
  - Over simplification of geometric model inside the sensor model (will consult Xiaojun for help)
  - Simulation time may take longer than realtime due to high density sensor such as Velodyne, and camera(high res)

#What's Next

- Processing Lidar's data
    - 3D velodyne, data accumulator, filtering, feature detection, cloud object association
    - 2D (Hokuyo/Scala), obstacle detection and tracking, behavior, test braking distance
- Processing Vision
    - Camera calibration
    - Visual servoing (for parking)
    - Vision-based slam (SeqSLAM)

#What's Next
- Backend
    - Mapping and simplified localization
    - Experiment with multiple modalities for localization
    - Tracking
    - Add suspension for realism (useful to simulate pitch/roll angle affect on sensors)
    - Integrate with Behavior and simulate various possible cases


# Thank you
Q&A
