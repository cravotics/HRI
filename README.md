# MULTI⦿VIZ

**A ROS2-based, user-friendly GUI for multi-robot monitoring, control, and experimentation**

---

## Table of Contents
- [Overview](#overview)
- [Project Outcomes](#project-outcomes)
- [Success Metrics](#success-metrics)
- [Analysis of Results](#analysis-of-results)
- [Demonstration](#demonstration)
- [Individual Contribution](#individual-contribution)
- [License](#license) *(TBD)*
- [Contact](#contact)

---

## Overview

**MULTI⦿VIZ** is a user-friendly GUI designed to effectively monitor, control, and experiment with multiple robots in real-time, using ROS2 (tested with ROS 2 Humble). It offers:
- Real-time visualization of odometry, LiDAR, and camera feeds.
- Teleoperation and waypoint navigation (x, y, yaw).
- Integrated voice command interface with Text-To-Speech feedback.
- A customizable "Experimentation" display (for depth maps, heat maps, segmentation, etc.).
- An emergency kill switch for instant robot shutdowns.
- Battery monitoring for real-world applications (when integrated with appropriate battery feedback topics).
  
This tool addresses the limitations of existing solutions like RViz for multi-robot handling and focuses on modularity, scalability, and open-source adaptability.

**Release Date:** January 1, 2025  
**Test Environment:** ROS 2 Humble | Gazebo (multiple TurtleBot Waffle robots)

---

## Project Outcomes

1. **Multi-Robot Visualization**  
   Real-time odometry, LiDAR, and camera visualization for multiple robots.

2. **Integrated Control**  
   - Teleoperation (manual control via GUI buttons).
   - Waypoint navigation (specify x, y, yaw).

3. **Voice Command + TTS**  
   Accurate voice command recognition with auditory feedback via Text-To-Speech.

4. **Custom Experimentation Display**  
   For advanced data visualizations such as depth maps, heat maps, segmentation results, etc.

5. **Emergency Kill Switch**  
   Immediate shutdown of all robots in motion, ensuring safety during operations.

6. **Smooth Multi-Robot Simulation**  
   Validated in Gazebo environment for real-time performance and coordination.

---

## Success Metrics

1. **Data Visualization Latency**  
   Less than 1000 ms delay in odometry, LiDAR, and camera feeds during multi-robot operation.

2. **Voice Command Accuracy**  
   ≥90% correct recognition of varied voice commands.

3. **Experimentation Display Reliability**  
   ≥90% success rate in displaying correct outcomes (e.g., heat maps, depth maps, etc.).

4. **Emergency Shutdown**  
   Immediate stop of all robots upon activating the kill switch.

5. **Real-Time Logging Performance**  
   No noticeable lag in logging or visual feeds.

6. **TTS Feedback Clarity**  
   Clear and understandable Text-To-Speech responses.

7. **Open-Source Modularity**  
   Verified code modularity and scalability for community collaboration.

---

## Analysis of Results

1. **Latency and Performance**  
   - Achieved <1000 ms latency in data visualization.  
   - Minimal noticeable lag when scaling to multiple robots.

2. **Voice Interaction**  
   - Surpassed 90% voice recognition accuracy using Python speech recognition libraries.  
   - TTS (via Pyttsx3) provided clear auditory feedback.

3. **Experimentation Visuals**  
   - Custom displays for depth/heat maps matched expected results in most test trials.

4. **Emergency Kill Switch**  
   - Instantaneous robot stoppage during multi-robot motion, confirming system safety.

5. **Scalability and Robustness**  
   - Post-test evaluations indicate readiness for open-source release and community collaboration.

6. **Conclusion**  
   - MULTI⦿VIZ demonstrates a robust, scalable GUI for multi-robot systems, meeting real-time operation demands.

---

## Demonstration

A brief demonstration of **MULTI⦿VIZ** can be viewed here:  
[**MULTI⦿VIZ Demo.mp4**](https://drive.google.com/file/d/1j6JT9HDtINcb3gtojFghZ0qUcL4dRSyY/view?usp=sharing)


---

### Key Responsibilities

1. **GUI Development (From Scratch)**  
   - Implemented initial and alpha versions in PyQt5.  
   - Ensured integration points for multi-robot system features.

2. **Teleoperation Module**  
   - Designed the manual control panel (buttons and sliders).  
   - Enabled control for multiple robots concurrently.

3. **Logging Mechanism**  
   - Added a real-time logging section to display executed actions, aiding in debugging.

4. **Modular & Scalable Code**  
   - Wrote the GUI with future expansions in mind.  
   - Ensured easy adaptation for additional robot types and features.

5. **Simulation Integration**  
   - Co-worked on the multi-robot Gazebo setup.  
   - Verified seamless communication between the GUI and simulated robots.

6. **System Architecture & Documentation**  
   - Documented system design after testing.  
   - Provided guidelines for future contributors and expansions.

---

## License
*(License information to be updated; consider using an open-source license like MIT, Apache 2.0, or GPL.)*

---

## Contact

For more details or collaboration:
- [Email](mailto:your_email@example.com)
- [GitHub Issues](#) *(Replace with actual GitHub repository link)*

---

**© 2025 MULTI⦿VIZ Contributors.**
