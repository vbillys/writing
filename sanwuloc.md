
---
title: Design and Structure Document for LIDAR Map-based Localization (Indoor Service Robot)
author: Saputra V.B.
date: May 9, 2016
---

\newpage

# Introduction  

In order for robot to successfully navigate throughout the environment indoor and outdoor, precise current position of the robot need to be found. The position of the robot is the basis for navigation control of the robot to reach certain goal point. For outdoor robots, GPS signal could be used. Nevertheless, in many indoor applications, robots need to learn a map of unknown or known environments and to navigate. The problem of finding positions of the robot over its travelling trajectory, and by the same result builds the map is called *Simultaneous Localization and Mapping* (SLAM). In addition, SLAM also closely related to *Moving Object Tracking* (MOT), as in SLAM it is preferable to distinguish between sensor readings whose originated from map static objects and dynamic moving object. By not including the readings from moving object, SLAM can be more accurate as its observation in mapping and localization is cleared from temporary readings. The whole package is called *SimultaneousLocalization and Mapping and Moving Object Tracking* (SLAMMOT) problem. We aim to solve SLAMMOT with lowest cost available but results should be as robust in the context of application at hand.

# Brief survey of State of the Art  

Related work using LIDAR based localization has been around from quite some time. The most popular is the *gmapping* which uses *Particle Filter* method to fuse robot odometry and laser observation. This approach needs fairly accurate odometry, does not leverage on high scan rates, and the published work is in 2D although some extension to 2.5D (multi-layered) and 3D is possible.  

Recent availability of 3D LIDARs, such as Velodyne Laser Scanners, made possible the development of 6D SLAM, that is localization done in 3D space that additionally includes roll, pitch, and height directions. Because of increased number of *Degrees of Freedom* (DoF), the search process is prone to starting guess in each iteration. Thus, usually accurate odometry with IMU is needed, and parameters must be cherry-picked carefully to achieve good result.  

Works in vision also recently has capitalize on the low cost of camera sensors, and recent advancements has brought some insight into how a monocular camera can also tracked up to 6 DoF. However, vision data will be always affected by lightning condition. This is promising to industry as the vision sensors are cheaper compared to LIDARs, and GPU based processing has recently gaining more popularity owning to its low power and enormous number of cores.  

Localization can also be done using both color and depth data. The *RGBD SLAM* method shows how a *Microsoft Kinect* and its variants can be used to acquire realtime localization in 6 DoF. It also can leverage on the GPU processing to fasten the sample rate. One of its weakness is that the sensor cannot work under direct sunlight. One way to improve is to replace the sensor with a stereo camera, but the depth computation will be much more intensive.

## Our Robust Low Cost LIDAR Map-based Localization  

The first objective is to use a 2D laser scanner system to achieve good localization inside an office environment. In order to do early experimentations for SLAM technologies, a commercially available but also open-sourced hardware named *Kobuki* is setup with a low cost *RPLidar* laser scanner. These can be depicted in Fig. \ref{fig:kobuki} and Fig. \ref{fig:rplidar}, respectively.

\begin{figure}
\centering
\includegraphics[scale=0.555555]{kobuki.jpg}
\caption{Kobuki experimental platform (for SLAMMOT testing).}
\label{fig:kobuki}
\end{figure}
\begin{figure}
\centering
\includegraphics[scale=0.555555]{rplidar_pic.pdf}
\caption{Low cost 2D laser scanner.}
\label{fig:rplidar}
\end{figure}

A simple wholistic structure of robot intelligence can be shown in Fig. \ref{fig:robot_struct}. The Navigation, SLAM and Perception subsystems are the foundation core of higher level command of the robot. From the High Level Control process groups consists of processes that allows robot to execute certain desired or programmed tasks, such as picking object or follow certain trajectories. It also includes a Human Machine Interface unit, that receives necessary input from human.  In this document, the problem of addressing structure of overall SLAM and MOT is focused. More technical details can be found in the corresponding reference.  

As can be seen, for the *Moving Object Tracking* (MOT) subproblem, is mainly handled in the Perception subsystem. This is because perception sensors do have more dense data that can achieve more accurate object detection and tracking. In the introduction above, the SLAM subsystem will benefit from the MOT solution in a way that sensor readings can exclude those moving object for accurate matching to the map.

The SLAM subsystem mainly uses Pose Estimation result from the Odometry and series of laser scans as the robot moves, to build knowledge of the environment (a map), and also to match the laser scan to the map (localization). The SLAM can be divided into two major steps: Mapping and Online Localization.

\begin{figure}
\centering
\includegraphics[width=.85\textwidth]{robot_struct.pdf}
\caption{Standard framework of Service Robots.}
\label{fig:robot_struct}
\end{figure}

### Two-steps approach for Mapping and Localization  

1. ***Mapping***  
   When designing robot for certain location, robot does not know what the environment looks like. The mapping step is to gather data from LIDAR and Odometry to build offline map that can be used for localization later. The structure is depicted in Fig. \ref{fig:mapping_struct}.  

   The Pose Estimation is firstly gathered from calculated values from IMU and motor encoder sensors. These values will give pure odometry. Because odometry pose will drift over travelling, we need to correct it using the more accurate 2D LIDAR laser scan data. The method that we use at the core is a variant of *Iterative Closest Point* for laser scan matching. This will give more accurate corrected Pose Estimation. This information not only corrects the pure odometry but also is used to build the map.  

   Building the map requires transforming the laser scan readings into the corrected Pose Estimation and thus putting the points into the correct structure of the map. Accumulation and Loop Closure pipelines (also shown in the figure), are needed when the environment is challenging, such as in the case of not much shape features that one scan cannot *see* much of them. Loop Closure is explicitly needed if the environment is at large scale in such a case where similar sensors readings could exist in two or more different localtions. Loop Closure can also be done manually offline to improve map accuracy. The map is stored in a map server, which acts as a database to record and retrive map information to all subsystems that require it.


\begin{figure}
\centering
\includegraphics[width=1.15\textwidth]{mapping_struct.pdf}
\caption{Pipeline for Mapping Mechanism.}
\label{fig:mapping_struct}
\end{figure}

2. ***Localization***  

After Mapping step, the online Localization step is running on the robot as long as the robot needs to navigate. The implementation is actually simpler than the Mapping step in a way that localization only needs to correct the Pose Estimation originating from odometry with the stored map. However, if environment changes, map needs to be updated. This issue will be addressed in the next subsections among other potential problems.

### Object Detection and Sensor Reading Filter for SLAM


\begin{figure}
\centering
\includegraphics[width=1.05\textwidth]{object_loc_struct.pdf}
\caption{Illustration of obstructed scan of the LIDAR, preventing accurate observation of the environment (map).}
\label{fig:object_loc_struct}
\end{figure}

One important issue needs to be addressed is that the Localization needs a clear view of the environment, both for building the map, and to match againts it. This is imperative so that the scan matching technique that is employed can accurately predict where the robot is with repect to the map. Thus, obstructed scan that ends at dynamic objects surfaces, should be excluded from Mapping and Localization. This is done by employing basic segmentation and classify using some learned features that can differentiate whether a point belongs to static and dynamics objects. This issue can be shown in Fig. \ref{fig:object_loc_struct}.



### Map Maintenance

As been discussed above, during non-stop operation, the robot may encounter some environment change, e.g. static objects may change positions. This will result that the map collected previously and processed offline not updated anymore. Thus the map must be maintained by updating it. The process is similar to the Mapping step elaborated above. However, we need to be selective in chosing what and when to update the map so that the localization will still be usable and accurate, although the map evolve from time to time.

### Combination with Vision-based Localization  

For even more robust solution, color data could be exploited. Vision data can be acquired from camera sensors, either with depth or not. The work related to vision SLAM is *ORBSLAM* and *RatSLAM*. Both are quite promising in term of robustness, but the *RatSLAM* is a simpler solution that can be combined with LIDAR-based SLAM. Rather than using cognitive technique to *learn* the environment in the *RatSLAM*, another approach is to just basically take fingerprints of various location that robot most likely encounter and store in a database. This would be comparable to a map, but the structure is different that in a way the data is images instead of point clouds. 

In sum, the usages of the Vision-based localization to complement the LIDAR-based are:  
\begin{enumerate}
\item{To initialize first location of the robot. Rich information of the most likely robot is located can come from vision.}
\item{To help to relocalize the robot if it get lost, as sometimes LIDAR can encouter ambiguous map structure that looks alike.}
\item{Complement LIDAR data in pixel-point level, that is to find corresponding color of each of the scan point from LIDAR. This can improve accuracy and robustness of the whole SLAM, but carefull calibration is necessary. To avoid calibration, an \textit{RGBD sensor} such as Kinect variants could be used.}
\end{enumerate}

- - - -  
  

Document Revision Control:

\begin{enumerate}
\item{Version 0.1}
  \begin{enumerate}
  \item{Timestamp : 9 May 2016}
  \item{Remarks : Initial Version}
  \end{enumerate}
\end{enumerate}


