% Progress Update
% Saputra V.B.
% Nov 03, 2015

# Overview

- Radar Development
- Simulation Development as platform to test perception and localization on COMS


# Radar Development

<section>
<br>
<br>
*Code is complete and Testing now*

  - There isn't any cable prob on esr final, but the data also not good
  - Will keep testing and collecting data
  - test on road, and while moving, with egomotion
  - the esr has maximum yaw angle, this would imply the need to change mounting angle
  - there is an error to be investigated, but lack of documentation
</section>

<section>
**Integration**

  - Xuxu's filtering, if he is around
  - Integration with PanYu's, using Sensor Fusion

**Mitigation**

  - hardware solution: may need to replace all with sms
</section>

<section>
**Conclusion**

  - Draft Alphard, to replace all radar with sms, integrate egomotion with ZW
  - Final Alphard, continue testing with the main team for integration and recover Delphi (if possible)
</section>


# Simulation

<section>
**As Development Platform**

  - Using V-REP (free, or can buy license if used for commercial purposes)
  - To test and choke algorithm stability without driver and other hardware specifics
  - Mitigate bugs and human errors early before real tests
  - Ideal for testing algorithm as data are without noises, and later we can compare with real data
</section>

<section>
**As Testing Tool**

  - Good for multiple robot, to verify interactions
  - Quickly prototype algorithm for new environment
  - hardware abstraction, decouple development, flip flag to switch
  - identify problem early, especially those related to fine tuning the algorithm
  - Visualization
</section>

# Simulation Early Results


# Thank you
Q&A
