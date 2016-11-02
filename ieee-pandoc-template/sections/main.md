# Introduction

Recent advancements in robotics technology have brought great benefit to robotic research. Since the Defense Advanced Research Projects Agency (DARPA) 2003 Grand Challenge [@ventures2006stanley], many research groups have focused their effort to solving problems related to autonomous vehicle navigation. As laboratories and research centers bringing the autonomous vehicle technologies to real world applications [@Ford_Jan_16], acceptance of fully unmanned vehicles will still be a challenge. However, with current progress it can be shown that for a specific problem domain, in this case service robots, they can be utilized for automation of human service oriented tasks, such as automated visitor recognition,  serving check in and check out, and auto vaccum-cleaner machine. This is supported by the growth of service robotics industry in the recent years [@SVR_Nov_15].  \citeauthor{ZhangHaojie7225673} [@ZhangHaojie7225673]  introduced mobile service robot for electrical power plant inspection. This in turn could reduce or replace human involvement in manual inspection process. \citeauthor{WeiJunqing6629559} [@WeiJunqing6629559] presented an autonomous driving car for research purposes that includes features that allows for urban road navigation. Similarly, \citeauthor{Broggi7225765} [@Broggi7225765] introduced a more advanced autonomous driving platform consisting of high performance cameras, LIDARs, and other navigation sensors. While these platforms are very attractive in term of their capability of handling big streams of sensors data, the demanding computing power renders them unsuitable for small service robots. The Autonomous Service Robots (ASR) proposed in this patent is developed as a prototype to service human basic needs with minimal number of sensors and offering a cheaper solution.


# Structure


Autonomous navigation of the ASR heavily depends on the perception system which data comes from various sensors and processed by IRL. In this case, there are three sensors used by IRL:


#### Laser Range Sensors
LIDAR ( Light Detection and Ranging) is one of the popular light range sensors used in various field of Robotics, Autonomous Vehicles, and Geo-mapping, forestry, etc. LIDAR, which stands for Light Detection and Ranging, is a remote sensing method that uses light in the form of a pulsed laser to measure ranges from IRL to the surrounding objects. The internal component of a LIDAR can be shown in Fig. \ref {fig_lidar_details}. LIDAR will provide the distance of the object by emitting laser towards it and analysing its reflection. The servo motor will direct the laser through the tilting mirror to cover the view angle of the LIDAR. Compared to stereo vision systems [@Camellini6856563], LIDARs generally has more field of view, longer detection range, and more accurate depth information. 

In the ASR, the LIDAR is mounted forward-facing and backward-facing of the robot. The backward-facing LIDAR, although is optional, is important to aid IRL and ASR navigation as it provides more view angle to surrounding objects. This setup can be depicted in Fig. \ref{fig_lidar_placements}.

#### Stereo Camera
For the big area, the IRL needs to detect objects not only using LIDAR, but also using images by means of camera sensors. The camera setup shown in Fig. \ref{fig_camera_placements} is to complement data from LIDAR for IRL to do the localization. The stereo camera is mounted higher near the head of the robot to cover high objects such as ceilings. The stereo camera should have at least 120 degree opening angle. The image features are projected to 3D point cloud by calculating the disparity of the keypoints between two cameras. In this way, the IRL can track robot position more accurately.

#### Upward-Looking Camera

In addition to the stereo camera, another camera which is upward-looking is proposed to track image features on the ceiling only. This is useful when the robot is operated in crowded environment, and using the image features on the ceiling could help IRL to keep track of the robot position.

\begin{figure}[!t]
\centering
\includegraphics[width=3.5in]{lidar_details.pdf}
\caption{Internal Diagram of the LIDAR.}
\label{fig_lidar_details}
\end{figure}


\begin{figure}[!t]
\centering
\includegraphics[height=1.7in]{lidar_placements.pdf}
\caption{IRL LIDAR locations on the ASR.}
\label{fig_lidar_placements}
\end{figure}

\begin{figure}[!t]
\centering
\includegraphics[height=2.5in]{camera_placements.pdf}
\caption{IRL Camera locations on the ASR.}
\label{fig_camera_placements}
\end{figure}

# Implementation



\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{block_diagram.pdf}
\caption{Summary of IRL.}
\label{fig_block_diagram}
\end{figure}

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{localize_2_highres.pdf}
\caption{Feature Detection and Particle Filter of the IRL process.}
\label{fig_localize}
\end{figure}

As opposed to outdoor localization, the environment where ASR usually deployed is indoor and Global Positioning System (GPS) data is unavailable, thus the name is Indoor Robot Localization (IRL). Conventional method to do robot localization in rather controlled environment is to modify or put line markers on the floor. This approach will allow one to control robot navigation by compensating lateral position with respect to the markers [@loc_Boss]. This allow the robot control to always offset its position to always stay on the predestined track. This approach often used in factories, however, for human service applications often we cannot modify environment and usually clients have their own established building that cannot be changed.

A hybrid approach combining both LIDAR and vision IRL is developed. This approach fuses both information from LIDAR, cameras and a prior map built. The method is based on Particle Filter or sequential Bayes filter that involves prediction from robot motion model and measurement update cycle [@LiT6550131]. The measurement update computed from odometry and the feature scan matching. This approach is suitable for service applications because the size of the map is ranging from small to medium scale up to a few kilometer squares. With the current computing technology, the map of this size can be fully loaded into memory.

Before deployment, the ASV needs build its feature map by collecting data during manual driving. A map is built by aggregating the data from laser range sensors installed in the ASR. This is implemented in two-staged process. In the first stage, raw scan data is collected and sequentially aligned with an Iterative Closest Point (ICP) method [@censi08plicp]. During alignment, raw point cloud map is built by registered point cloud and any loop closure will improve consistency of the detected robot positions during mapping [@tiar2015fast]. In the second stage, obvious features such as walls and large objects are verified to confirm robot can use these features and prevent IRL from total lost (i.e. lose tracking of the robot current position).  The result of feature detection, and the ASR Particle Filter map based localization can be shown in Fig. \ref{fig_localize}.


As mentioned by \citeauthor{MurArtal7219438} [@MurArtal7219438] monocular Visual SLAM suffers from large initialization error, thus the proposed stereo camera does not have this problem. Furthermore as the LIDAR based IRL outputs point cloud data, with stereo camera 3D environment can be built. The IRL includes the vision localization into its particle filter evolution during robot movements. If the LIDAR cannot detect enough features to localize, the confident factor decreases, and it tries to track more features using vision. Thus, a dual strategy is implemented, which is robust than LIDAR based localization alone. IRL requires no modification of the environment which is an advantage compared to those markers based conventional methods. Fig. \ref{fig_block_diagram} summarizes the IRL procedure.


# Conclusion

Depending on the specific task, various tools which include sensors and/or actuators that are necessary for the service task can be mounted onto the ASR. These sensors and actuators are collectively called as a workload. The ASR modules are customizable to carry the workload and perform the desired service task. The ASR localize itself by using information from map and the sensors, thus eliminating the need for installing environmental guidance such as RFID [@ZhangHaojie7225673] or magnetic rails [@LuShouyin5354591].

The ASR has been developed to handle human service tasks. It has generic architecture which is reproducible for other purpose of use. During the development of the ASR, issues had surfaced, and dealt with current approaches. The main function of the ASR is to navigate through set of prior taught waypoints. Precise IRL is achieved by combining LIDARs and cameras. This allows adaptation to various human oriented service tasks, with appropriate tooling mounted. The experimental prototype results verified that the ASR is able to execute the path reliably and safely. 


<!--
That’s all folks!
-->
