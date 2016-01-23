# Introduction

Recent advancements in robotics technology have brought great benefit to autonomous vehicle research. Since the Defense Advanced Research Projects Agency (DARPA) 2003 Grand Challenge [@ventures2006stanley], many research groups have focused their effort to solving problems related to autonomous vehicle navigation. As laboratories and research centers bringing the autonomous vehicle technologies to real world applications [@Ford_Jan_16], acceptance of fully unmanned vehicles will still be a challenge. However, with current progress it can be shown that for a specific problem domain, in this case service vehicles, the autonomous vehicle can be utilized for automation of human service oriented tasks, such as automated road cleaning,  grass cutting and leaves sucking machine. This is supported by the growth of service robotics industry in the recent years [@SVR_Nov_15].  \citeauthor{7225673} [@7225673]  introduced mobile service robot for electrical power plant inspection. This in turn could reduce or replace human involvement in manual inspection process. \citeauthor{6629559} [@6629559] presented an autonomous driving car for research purposes that includes features that allows for urban road navigation. Similarly, \citeauthor{7225765} [@7225765] introduced a more advanced autonomous driving platform consisting of high performance cameras, LIDARs, and other navigation sensors. While these platforms are very attractive in term of their capability of handling big streams of sensors data, the demanding computing power renders them unsuitable for small service vehicle. The Autonomous Service Vehicle (ASV) proposed in this paper is developed as a prototype to demonstrate autonomous vehicle capability tasked for a service vehicle.

Autonomous platforms are usually retrofitted from manually driven vehicles. This approach is commonly used because it is the most viable and affordable way to install a Drive-By-Wire (DBW) system into existing vehicle rather than to entirely build a new one. It was also studied that automation driver’s task has positive impact to improving driving safety [@stanton1996fly]. Various effort to assist drivers has lead to development of ADAS system [@6232125;@7225760]. A DBW enables computerized control of the vehicle by means of replacing human drivers with actuators, which are usually driven by electrical motors [@1035218]. The ASV presented in this paper is retrofitted from a compact electric vehicle [@SongZW_IV_2015], and thus we studied the feasibility of converting this class of vehicles into service vehicles. 

This paper contribution is threefold. First, to describe and propose a generic design of the system architecture based on necessary components for the ASV applications. Second, to propose the ASV architecture to be adopted into generic service platform for both research and application purposes. Although, the implementation of the ASV presented here are subject to certain sources, the components are generic and should be reproducible with similar effort. Lastly, to present discussion on issues surfaced during tests and evaluation of the ASV navigation in urban road environment.

# Architecture

The main service types for ASV are related to defense, agriculture and inspection, logistics and medical applications [@SVR_Nov_15]. As a platform, ASV is required to navigate through environment to complete the assigned service tasks. The approach taken is to modify current vehicle to enable automated control by means of processing sensors data. In this paper, the ASV components are designed to complete a case of urban road navigation. Components of the proposed ASV can be summarized in Fig. \ref{fig_1}. 

<!--
The system heavily depends on various types of sensory data and sensors of the same type are synced with one another through data sync modules. To make sense of sensor data, raw data processing is required for different types of available data. Overlapping field of views from different sensors could be combined and the processing result is fused through a Low Level Fusion module. This module also does some filtering to eliminate local noises and thus, make the information more robust to small disturbances. The output objects and localization data from low level fusion modules are then sent to a High Level Fusion Module whereby all objects and filtered sensor information are fused together to form a world model.
-->





\begin{figure}[!t]
\centering
\includegraphics[width=3.5in]{ecoms_arch.pdf}
\caption{Block Diagram of the ASV system.}
\label{fig_1}
\end{figure}



## Middleware
Middleware refers to the software component that connects the various modules across network of distributed system. It makes communication transparent to connected modules by providing reliability, consistency, security,   and privacy capabilities. In the ASV architecture CHROMOSOME (abbreviated as XME) [@CHROMOSOME], a message-oriented middleware is used. XME implements the publish-subscribe message paradigm, which provides guarantees with respect to the behaviour of distributed applications [@buckl2014chromosome]. The other  alternative middlewares are Robotics Operating System (ROS) [@quigley2009ros] and  OpenRTM-aist [@ando2005rt] which are commonly used in the field of robotics. 


## Localization

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{localize_2.pdf}
\caption{Feature Detection and Particle Filter of Localization process.}
\label{fig_localize}
\end{figure}

As opposed to indoor localization, the environment where ASV usually deployed is outdoor and GPS data is available. However, it is studied that GPS data is often contain errors that is too large for autonomous navigation [@6629559]. One approach to compensate the error is to use local lane marker or road boundary information to adjust lateral position of the vehicle with respect to the road [@loc_Boss]. This allow the vehicle control to always offset the vehicle position to always stay on the drivable area on the road. However, it can be argued that the position in longitudinal direction along the road with this approach is not accurate. For service applications, it is necessary that both lateral and longitudinal position of the vehicle are determined accurately as the ASV often need to stop at specific point in its route. We developed a hybrid approach that fuses both information from GPS and a prior map based online localization. The method is based on Particle Filter or sequential Bayes filter that involves prediction from vehicle motion model and measurement update cycle [@6550131]. The measurement update computed from GPS and the feature scan matching. This approach is suitable for service applications because the size of the map is ranging from small to medium scale up to a few kilometer squares. With the current computing technology, the map of this size can be fully loaded into memory.

Before deployment, the ASV needs build its feature map by collecting data during manual driving. A map is built by aggregating the data from laser range sensors installed in the ASV. This is implemented in two-staged process. In the first stage, raw scan data is collected and sequentially aligned with an Iterative Closest Point (ICP) method [@censi08plicp]. During alignment, raw point cloud map is built by registered point cloud and any loop closure will improve consistency of the detected vehicle positions during mapping [@tiar2015fast]. In the second stage, features along lateral and longitudinal direction are gathered and registered into processed feature map. The dominant features used for urban environment are road curbs and tree trunks. The result of feature detection, and the ASV Particle Filter map based localization can be shown in Fig. \ref{fig_localize}.
<!--

## Mapping

Procedure of mapping includes:

1. Point cloud assembly
2. Point cloud registration
3. Extracting useful information

### Point cloud assembly

The assembly process is aggregating several point cloud frames from sensor scans with some process of point cloud matching and registration.

### Point cloud registration


### Extracting useful information

* Road curb
* Traffic signs/lights
* Building features

#### Road curb

Road curb is the most reliable feature in road boundary detection. It is easy to detect and currently it is the baseline for localization.

#### Traffic signs/lights

#### Building features



The localization is done by modeling the problem using *Gamma function* satisfying $\Gamma(n) = (n-1)!\quad\forall n\in\mathbb N$ is via the **euler** integral:

\begin{equation}
\label{eqn_example1}
x = \sum\limits_{i=0}^{z} 2^{i}Q
\end{equation}

\begin{equation}
\label{eqn_example2}
\Gamma(z) = \int_0^\infty t^{z-1}e^{-t}dt\,.
\end{equation}

\begin{equation}
\label{eqn_example3}
\mathbf{V}_1 \times \mathbf{V}_2 =  \begin{vmatrix}
\mathbf{i} & \mathbf{j} & \mathbf{k} \\
\frac{\partial X}{\partial u} &  \frac{\partial Y}{\partial u} & 0 \\
\frac{\partial X}{\partial v} & \frac{\partial Y}{\partial v} & 0
\end{vmatrix}
\end{equation}

Equation (\ref{eqn_example2}) is convoluted by means of *Gaussian Distribution*. In other words, (\ref{eqn_example1}) depicts the spectrum of (\ref{eqn_example2}). The **eigen **matrix for the point cloud distribution is given in (\ref{eqn_example2}).
-->


<!--
## Perception

(LIDAR) based, but camera just for logging and teleop
-->

## Obstacle Detection
During operation of the ASV, obstacle obstructing or near the vehicle should be considered. Thus, by using a laser range sensor model for detection, a set $L$ of $n$ points which each has a range $r_{i}$ and a angle $\alpha_{i}$, measured from laser beams $L=\left\{(r_{1},\alpha_{1}),...,(r_{n},\alpha_{n})\right\}$ are clustered to extract objects [@Juric-Kavelj2008_407]. A Gaussian smoothing is applied to remove spurious noise before processing. Two subsequent points belong to one cluster satisfies the distance threshold  $d(r_{i},r_{i+1}) \leq D_{0}+D_{1}min(r_{i},r_{i+1})$, where $D_{0}$ is the accuracy of the range data. $D_{1} = \frac{3}{2}(\alpha_{i+1}-\alpha_{i})$ is a constant that determine maximum change allowed. Point that break the threshold will be the start of another cluster. Processed laser scanner is represented by a set of clusters $\left\{ C_{1},...,C_{m} \right\}$. Each cluster consists of tuples $({}^{C}r_{i},{}^{C}\alpha_{i},{}^{C}x_{i},{}^{C}y_{i})$ where ${}^{C}x_{i}$, ${}^{C}y_{i}$ are the Cartesian coordinate representations.

Each cluster is a potentially detected object. Based on the lateral proximity of each cluster, object boundaries are found by determining if each cluster satisfies $w_{min} \leq d(y_{min},y_{max})  \leq w_{max}$, where $w_{max}$ and $w_{min}$ depend on widths of object of interest. The detected obstacles are sent the path planner to aid in calculation of the navigation course. Moreover, obstacle detection improves false recognition removal of map objects for localization [@5164269]. The cluster points of detected objects are used to determine areas of where measured points should be excluded from the localization and mapping process.



## Path Planning

The Path planning module generate reference trajectory for the ASV to follow from a start to a destination point. The ASV path planning is implemented in two stages: Global Path Planning and Local Path Planning. 

### Global Path Planning
<!--
Global path planning aims to generate the global waypoints to reach destination from origin point. Global path planning can be achieved by notable path planning algorithm such $A^{*}$, $D^{*}$, Rapidly exploring random map, probabilistic roadmap. In the case of the ASV, global path planning is generated by manual drive and record the UTM coordinates for path driven.  Recorded path are later downsampled by post processing Alg. \ref{alg_global_waypoints} for saving memory and path optimization. During post processing duplicate UTM points, straight line points, points within vehicle turning radius are downsampled.  For a typical example, if original recorded waypoints could contain ~35000 UTM points, after post processing the global waypoints will contain ~100 UTM points.  
-->
Global path planning aims to generate the waypoints to connect the destination and the start point. In the literature, global path planning can be achieved by the notable path planning algorithm such $A^{*}$[@hart1968formal], $D^{*}$[@stentz1994optimal], Rapidly-Exploring Random Trees[@lavalle1998rapidly], Probabilistic roadmaps[@kavraki1996probabilistic]. For the ASV, a teach and repeat solution is prefered as the planned path rarely change. This implies that once the ASV is taught to navigate through certain waypoints, it will be able to repeat it and more complex routing can be achieved by combining multiple sets of waypoints. The ASV obtain its trajectory by recording the vehicle position provided by localization during manual driving. The trajectory obtained is further optimized by post processing Alg. \ref{alg_global_waypoints}. In post processing duplicate vehicle position points, points lie on straight line, curved points within vehicle turning radius are removed to generate the optimized path. 

<!--
For a typical example, if original recorded waypoints could contain ~35000 vehicle positional points, after post processing the global waypoints will contain ~100 vehicle positional points.  
-->

\begin{algorithm}
      \scriptsize
      \algsetup{linenosize=\scriptsize}
      \caption{Global Path Planning}
      \label{alg_global_waypoints}
      \begin{algorithmic}[1]
          \REQUIRE minLen, minRad, recordedPoint[] \COMMENT{array contains x, y and $\theta$}
          	\STATE initialize points[], globalPoints[] \COMMENT{array contains x, y and $\theta$}

          	\STATE totalGlobalPoints = 0
          	
          	\FORALL{$points$ in $recordedPoint$}
          		\IF{$totalGlobalPoints$ == 0}
          			\STATE append $points$ to $globalPoints$
          			\STATE $totalGlobalPoints++$
          		\ELSE
          			\IF{((distance between $points$  and $globalPoints[totalGlobalPoints - 1]$) $\geq$ $minLen$) $\AND$ (($\arctan$ between $points$ and $globalPoints[totalGlobalPoints - 1]$) $\geq$ minRad)}
          				\STATE append $points$ to $globalPoints$
          				\STATE $totalGlobalPoints++$
          			\ENDIF
          		\ENDIF
			\ENDFOR
      \end{algorithmic}
\end{algorithm}




### Local Path Planning
Local Path Planning aims to generate dynamic reference trajectories with vehicle dynamics constraints such as turning radius and maximum velocity. Based on Global Path Planning trajectory, Local Path Planning is recursively generated during vehicle on motion with specified interval to achieve smoother vehicle navigation. Local Path Planning mainly helps to deal with dynamic circumstances, such as avoiding obstacle along the global path and to perform smoother turning based on velocity constraint.
The Local Path Planning module is developed with Dubins-Curve [@DubinsCurves] library  which is based Dubins [@dubinscurve1957@] path method. Dubins path is one of the simplest geometric methods to compute the shortest path between two points in the Euclidean plane with turning radius as constraint. Dubins path is suitable for nonholonomic wheeled mobile robot or car like autonomous vehicles. Dubins path is built on top of the assumption that non-holonomic vehicle has a typical motion of going straight (S), turning left(L) and turning right(R). According to Dubins [@dubinscurve1957@] optimal path will be always one of the six possible motion sequences: {LSL, RSR, RSL, LSR, LRL, RLR}. For details on Dubins method, readers can refer to section 15.3.1 in [@lavalle2006planning].
Local Path Planning Alg. \ref{alg_local_waypoints} is executed at regular interval when the AVS has travelled a certain distance or any obstacle is detected along the path. In the event of obstacle obstructing the global path, the local path will be shifted to the left or right side of obstacle boundary based on environmental factors for safer avoiding. However, if the obstacle is an moving object, the planner is unable to avoid. Thus, in this case velocity profile will be lowered down based on a safe distance threshold. An example of the Local Path Planning avoidance is shown on Fig. \ref{veh_avoid_close}.

\begin{figure}[!t]
\centering
\includegraphics[width=2.0in]{vehicle_travelled_crop.pdf}
\caption{Vehicle avoidance path generated by local path planning}
\label{veh_avoid_close}
\end{figure}

\begin{algorithm}
	  \scriptsize
     \algsetup{linenosize=\scriptsize}

      \caption{Local Path Planning}
      \label{alg_local_waypoints}
      \begin{algorithmic}[1]
          \REQUIRE currentPos, obstaclePoints[], globalPoints[], turnRadius, stepSize, maxLen
          	\STATE initialize localPoints[], startPoint[], endPoint[] \COMMENT{array contains x, y and $\theta$}
          	\STATE wayLen = 0
			
			\STATE $globalClose \leftarrow$ Find closest $globalPoints$ to $currentPos$ \COMMENT{closest points based on Euclidean distance}
			
			\STATE $startPoint$ = $globalPoints[globalClose]$			
			
			\WHILE{$startPoint$ is not a last globalPoint $\AND$ $wayLen \leq maxLen$ }
				
          		\STATE $endPoint$ = $globalPoints[startPoint + 1]$
			
				\IF{$obstaclePoints$ exists between $startPoint and endPoint$}
          		\STATE $endPoint \leftarrow$ will be shifted point where obstaclePoint exist. \COMMENT{Based on environment path shift will either left or right}
          		\ENDIF           		
          		
          		\STATE $wayLen \leftarrow$ add Dubins Path Length for $startPoint$ and $endPoint$
          		\STATE $localPoints \leftarrow$ append Dubins Path for $startPoint$ and $endPoint$ with given $turnRadius$ and $stepSize$
          		\STATE $startPoint$ = $endPoint$
          	\ENDWHILE
          	
      \end{algorithmic}
\end{algorithm}

## Control    
Vehicle control performs necessary action to follow the intended path generated by path planner for smooth navigation towards destination point. The gas and brake control for regulating vehicle velocity along longitudinal direction will be handled by the velocity control module. The steering control for turning vehicle along lateral direction will handled by the lateral control module.

### Velocity control    
<!--
Velocity control will estimate the gas, brake motor position in the drive by wire system for maintaining the desired velocity and braking for the vehicle. The desired velocity of the vehicle is determined with the modules input such as path planning, obstacle detection and health monitoring system. 
For autonomous lateral control the fixed constant velocity will not be a wise choice, since the navigation path is not always straight line of travel. The path may consists of various turns and unexpected circumstances like obstacles. For smoother path following and avoid jerkiness of vehicle, speed profiling have to done based on path curvatures and speed limits. Based on the data collected by manual driving, the speed is calculated for various path curvatures. During the local path generation, the curvature of path is calculated and desired speed will be assigned to each curvatures across the path. To avoid rapid speed change and to make velocity transition smooth from straight line to curvature areas velocity averaging is performed in the direction of end to path start.
-->
Velocity control controls the vehicle velocity for along longitudinal direction by controlling the gas, brake motor position in the drive by wire system. Velocity control aims to achieve the desired velocity determined by the module’s input such as path planning, obstacle detection and health monitoring report. 
For smoother control and to avoid jerkiness of the vehicle, speed profiling is needed. Speed profiling is the process to estimates the speed based on the curvature of the path and speed limits for specific environments along vehicle trajectory. The estimation is calculated from the aggregation of the data collected during manual driving and maximum/minimum speed limits are applied. During local path planning, speed profiling is computed to determine the optimal speed. 


<!--
Apart from speed determined by speed profiler across the path, in event of obstacle detection vehicle will also perform action like slow down or stops based on the obstacle proximity to vehicle. In the event of certain module error or network communication  failure, health monitoring system will adjust the speed based on determined severity.
-->

To achieve the desired velocity, a closed loop Proportional-Integral-Derivative (PID) controller is implemented to compute the desired gas/brake position of the DBW motor. The closed loop PID controller helps in noise rejection and robustness of control with design flexibility. 
Based on desired velocity, the controller will calculate the motor position and passes to drive by wire (DBW) system. The wheel encoder measures the speed precisely and provide feedback to the controller. The block diagram of the velocity control implementation is shown in Fig \ref{fig_vel_control}. The governing control law of the PID controller is given in the following:

\begin{equation}
\label{eqn_pid}
GB = k_p{E} + k_{i}\int_{0}^{t}{E}\,{dt} + k_{d}\frac{dE}{dt}
\end{equation}

Where $GB$ is gas-brake position value, $E$ is the velocity error, $dE$ is the differential error, $k_p$ is the proportional gain, $k_i$ is the integral gain, $k_d$ is the differential gain.

\begin{figure}[!t]
\centering
\includegraphics[width=3.5in]{pid_velocity.pdf}
\caption{Velocity control based on closed loop PID.}
\label{fig_vel_control}
\end{figure}


### Lateral Control

Lateral control aims to control the steering angle of the vehicle based on trajectory generated by path planner. The lateral controller is developed based on Stanley’s [@ventures2006stanley] steering controller. Stanley steering controller is the geometric path tracking approach used by Stanford university’s autonomous vehicle during DARPA grand challenge 2006. 
The closed-loop geometric tracking controller helps vehicle to follow the desired path generated by local path planning. The controller is based on non-linear cross track error $C_{e}$ and heading error $\theta_{e}$. Cross track error is the lateral distance between front axle center point $(P_{x}, P_{y})$ and closest reference point $(R_{x}, R_{y})$ for which exponential can be shown [@ventures2006stanley]. This term helps the vehicle to steer towards the path to yield $C_{e}$ as zero when its following the path. 
The heading error $\theta_{e}$ is calculated by difference in vehicle heading $\theta$ and reference heading $\theta_{r}$ of path at $(R_{x}, R_{y})$. This error term directly applies to steering angle $\delta$ with some steering angle limitation.


\begin{equation}
\label{eqn_heading_error}
\theta_{e} = \theta - \theta_{r}
\end{equation}

When the cross track error $C_{e}$ is non-zero, it adjusts $\delta$ such that the intended trajectory intersects the reference path tangent from $(R_{x}, R_{y})$ at velocity in time $V_{t}$ units from the front axle  center point $(P_{x}, P_{y})$. The resulting steering control law by combining the two terms are shown in (\ref{eqn_stanley_equation}). $k_{h}$ and $k_{c}$ are the gain constants for heading error and cross error, respectively. Fig. \ref{fig_stanley} illustrates the geometric relationship of control parameters.

\begin{equation}
\label{eqn_stanley_equation}
\delta(t) = k_{h}\theta_{e} + tan^{^{-1}}\left (\frac{k_{c}C_{e}(t)}{V_{x}(t)}  \right )
\end{equation}


\begin{figure}[!t]
\centering
\includegraphics[width=1.5in]{stanley_2.pdf}
\caption{Stanley controller geometry}
\label{fig_stanley}
\end{figure}










## Health Monitoring

Safety is very important in a critical mission project like autonomous service vehicle. Not only that robust operation is required, but also safety must be prioritized in the event of malfunctions or catastrophic failures. Health monitoring module is a component in the ASV that acts as a safety agent to react to those events in timely manner. It is based on publish-subscribe model. This allows both centralized and distributed monitoring model. In the initialization stage, each ASV functional module component subscribes to a health topic proceeding completion of self-diagnosis. The health topic is provided by the Central Health Monitor (CHM). In the operational stage, modules are to send unique description and event identification with timestamp to the CHM. 
The CHM is responsible for reporting any important event to the user, log the event, and take suitable action based on severity of the detected failure or error. In addition, each module has their own reporting module, but limited only to each module scope of operation. The severity levels and the corresponding actions can be summarized in Table \ref{table_risk_hm}. 

\begin{table}[!t]
\renewcommand{\arraystretch}{1.3}
\caption{Severity Level in Health Monitoring}
\label{table_risk_hm}
\centering
\footnotesize
\begin{tabular}{p{1.5cm}||p{6.cm}}
  \hline
    \textbf{Severity} &
    \textbf{Actions} \cr
  \hline\hline
    None & Normal Operation. Status Logging. \cr
  \hline
    Warn & Display warning on user interface. Status logging. \cr
  \hline
    Abort & Vehicle aborts mission, plans and stops at a safe zone. Status logging and displaying on user interface. \cr
  \hline
    Emergency Stop & Vehicle aborts mission immediately. Status logging and displaying on user interface. All modules restart and perform self-diagnosis. \cr
  \hline
\end{tabular}
\end{table}


## Human Machine Interface
Human Machine Interface (HMI) is a graphical user interface where users could control and monitor the ASV. We have developed a web based HMI, so that the ASV could be monitored and controlled from any remote location. Remote control functions include emergency stop immediate request, task scheduling and change or reroute the ASV global path plan.
The HMI is web based and developed with websocket [@fette2011websocket] protocol implementation for achieving low-latency, long-running  bidirectional communication. For security concern, HMI can be accessed only from intranet with unique private IP assigned for the ASV and user need to authenticate with username and password. Any HTML5 supported browser can be used to access the HMI. The HMI consist of dashboard, system interface and settings interface. They individually has different scope of information and options.
For example, Fig. \ref{fig_hmi} shows the HMI dashboard. The dashboard is essential for vehicle monitoring information such as precise ASV position on the map, vehicle speed, battery level and system log for displaying critical error logs and network status. The system interface is used to manage individual modules. It has options to starting, to stop and to view error logs. The settings interface is used to modify ASV parameters such as maximum speed, braking distance, velocity control and lateral control gain.




\begin{figure}[!t]
\centering
\includegraphics[width=3.0in]{hmi.pdf}
\caption{HMI dashboard interface}
\label{fig_hmi}
\end{figure}


# Experimentation

<!--
Fig. \ref{coms1_bare} shows the first vehicle platform that we first retrofitted for the autonomous vehicle project.
-->

In our previous work [@SongZW_IV_2015], we retrofitted the first version of a compact electric vehicle. In this paper, we upgraded the ASV system and improve installation by moving into the second version of the vehicle. The second version vehicle provides better stability in term of maneuverability, and provide more space at the back of passenger seat to install complete power supply system and necessary computing devices. The current ASV prototype is shown in Fig. \ref{coms1_bare}.

\begin{figure}[!t]
\centering
\includegraphics[height=1.7in]{coms2_1.pdf}
\caption{Vehicle platform for the ASV.}
\label{coms1_bare}
\end{figure}

<!-- \includegraphics[width=2.5in]{coms2_zw.jpg} -->

## Setup

Placement of the components on the autonomous vehicle can be seen in Fig. \ref{ipe_ex}, and their connection to the system can be summarized in Fig. \ref{dia_ex}. As can be seen, the vehicle has a dedicated box originally tailored for carrying load. This box is revamped so that it has more space to contain all the components. The ASV system is integrated through a Central Processing Unit (CPU) and a Sensor Actuator Hub (SAH). Both CPU and SAH are compact PCs, but for different purpose. All the modules including HMI and CHM are run on the CPU. SAH gathers data from sensors and from the Vehicle Interface (VI). On top of that, it also bridges the CPU with the DBW control and emergency stop signal can bypass control to instruct immediate brake. In order to provide real-time logging and a database server system is installed in a low-end compact PC. This log database server receives data from all modules, and writes to storage in timely manner so that in the event of sudden power loss or vehicle breakdown, most recent data can be recovered and analyzed.

\begin{figure}[!t]
\centering
\includegraphics[height=1.6in]{ipe_ex.pdf}
\caption{Placement of components in the ASV.}
\label{ipe_ex}
\end{figure}


\begin{figure}[!t]
\centering
\includegraphics[width=1.9in]{dia_ex.pdf}
\caption{Diagram of components connection.}
\label{dia_ex}
\end{figure}



 


### Drive-By-Wire

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{Overview.jpg}
\caption{Drive-by-Wire installation of the ASV.}
\label{fig_DBW}
\end{figure}

The original vehicle comes with steering and gas/brake pedals. The Drive-By-Wire (DBW) refers to the mechanics added on to the vehicle so that it can be controlled by computer. Two motors are installed, each for controlling the steering and the pedals. The steering actuation is achieved by modifying the steering column and combining it with the steering motor shaft. The pedals actuation is designed so that only one pedal is pressed at one time. This actuation is achieved by installing two levers on top of each pedal, and connecting them to the second motor: one lever via direct linkage, and the other lever via a metal wire in such a way that individual pedal can be pressed by selecting motor rotating direction. This removes the need for a third motor, and also increases safety that it is mechanically impossible to press both pedals in case there is error in the motor controller. The electric vehicle only has two gear options linked directly to the internal motor inverter as it does not have mechanical gear, but the gear is needed to determine the direction of travel. Thus, the gear selection can be controlled using electrical signal through the VI.

### Power Distribution	

\begin{figure}[!t]
\centering
\includegraphics[width=2.2in]{power_diagram.pdf}
\caption{Electrical diagram of the ASV.}
\label{fig_power}
\end{figure}

The power supply for the CPU, DBW, sensors and other peripherals is provided by a separated rechargeable Lithium Polymer (LiPo) batteries. The autonomous operation runs independently on the LiPo batteries to avoid complication that may arise if the main batteries is tapped. Furthermore, power supply for ASV is crucial as the autonomous system depends on it. Therefore, additional Power Supply Units (PSU) and a UPS are installed such as shown in Fig. \ref{fig_power}. There are multiple redundancies power line to secure fail-safe and fault-tolerant operation of the ASV during its navigation. In addition, VI unit is made of multiple stack of microcontroller and relay boards in order to ensure safe interfacing to/from the vehicle. Fans are installed to provide regulate temperature inside the box.

### Sensors
Autonomous navigation of the ASV heavily depends on the perception system which data comes from various sensors.


#### Odometry
<!--
Odometry is a method to fuse the motion sensors data to obtain the position of any mobile robots. 
-->

Odometry position is estimated via accumulating the motion sensors data over time. Pure odometry position is not recommended for  long-term navigation, since the motion sensor error will accumulate drift in the actual position over time. The navigation relies on the localization module to fuse odometry data with GPS data and the developed map based localization. The odometry position is derived from the equation \ref{eqn_simple_odometry}. The following two motion sensors are installed in the ASV: a wheel motion sensor (Kistler Wheel Pulse Transducers) to calculate vehicle travelled distance, and GPS/IMU sensor (Xsens MTi-G-700 GPS) to measure global positions and heading rate. 

\begin{equation}
\begin{split}
\label{eqn_simple_odometry}
\theta_t = \theta_{t-1} + \Delta\theta_{t} \\
x_t = x_{t-1} + (\Delta v_t + cos(\theta_t)) \\
y_t = y_{t-1} + (\Delta v_t + sin(\theta_t))
\end{split}
\end{equation}

where $\Delta\theta$ is the yaw rate obtained from IMU, $\theta$ is the heading, $\Delta v$ is distance obtained from wheel motion sensor, $x$ and $y$ are the Cartesian coordinates of the odometry position, and $t$ is the time index which is increased every sampling time.

#### Laser Range Sensors
LIDAR ( Light Detection and Ranging) is one of the popular light range sensors used in various field of Robotics, Autonomous Vehicles, and Geo-mapping, forestry, etc. LIDAR will provide the distance of the object by emitting laser towards it and analysing its reflection. Compared to stereo vision systems [@6856563], LIDARs generally has more field of view, longer detection range, and more accurate depth information. A four layers Lidar (Ibeo ScaLa) is mounted on front bumper facing forward for obstacle detection up to 100 meters. The obstacle detection can be done in one layer, but the other layers will provide redundancies if the ASV tilt forward or backward. In this case, one or more layers can hit the ground or the free space below or above an object, respectively. At least one layer will detect an object if there is one. Another 16 layers Lidar (Velodyne Puck) which can sense surrounding object and environment shape in 25m radius is used mainly for localization. This LIDAR is mounted roughly $30^{\circ}$ towards ground purposely for detecting main features of the environment such as the road curbs and the tree trunks which are used for localization.


#### Camera
The camera (Logitech C930e) is used for video data logging and remote monitoring via the HMI interface. Camera video frames are read using \emph{video4linux} API, which later encoded and streamed in \emph{WebM} format with \emph{ffmpeg} and \emph{ffserver} setup. We did try to use image processing to detect lane marker for navigation, but unfortunately due to shadows and significant change in ambient light level during the day, detection results were not stable. This issue can be solved by replacing the camera with HDR camera or fuse the results with the LIDAR point cloud intensity data. 
 
### Safety System
Safety is a primary concern for any autonomous system, AVS should be stopped safely in any unexpected circumstances, such as sensor failures or errors in software modules. In the AVS experiment, we have implemented safety in three levels. First level is implemented in the Health Monitoring system which monitors hardware, software modules and takes necessary action on failures reported. The on-board emergency switch which provides the second level of safety, can be accessed while there is a person inside the ASV. Third level is implemented by a remote software emergency stop through the HMI interface and a hard-wired radio emergency stop system which works on 900MHz and 2.4GHz frequency.


## Testing Results and Evaluation
Testing is carried out in an urban environment for the distance of 1km range. As explained in the global path planning, waypoints record by manual driving and processed. Figure \ref{map_waypoint} shows the map of the urban environment road structure and global trajectory for testing. For autonomous driving, the maximum velocity is capped by 4m/s and static obstacles are placed in 3 locations as shown on Fig. \ref{veh_path_overview}.
Figure \ref{vel_error_test} shows the output of velocity control with respective to desired velocity. Velocity controller achieves desired velocity linearly and little noise is also observed during constant velocity. Between 900 to 950 cycle time there is a sudden drop of velocity is noticed, which is caused by dynamic obstacle along the path that caused the vehicle to slow down. However, the ASV velocity controller can recover speed to the desired profile after the object is no longer detected.


Figure \ref{control_error_test} shows the output of lateral control such as angle error, lateral distance error (cross-track error) and final steering output. The control parameter of the ASV is tuned to tolerate localization error up to 20 cm, thus the final steering output follows angle error closely unless there is a steady-state lateral distance error. This can be seen that eventhough the localization emits episodic high-frequency noisy data due to bad perception or GPS signal, the controller is able to filter the steering output. This results in smooth driving experience if someone is inside the ASV.  In the event of turning in a higher error is observed. This is due to maximum steering constraint which is set to 0.41 radians. Errors during turning is expected to turn the steering to the direction of the path.

<!--
Figure \ref{veh_path_overview} shows the vehicle traveled path during the testing respective to reference trajectory. 
-->







 
\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{map_overlay.pdf}
\caption{Recorded Waypoint to travel}
\label{map_waypoint}
\end{figure}
 

 
\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{vehicle_travelled_label.pdf}
\caption{Vehicle path overview}
\label{veh_path_overview}
\end{figure}
 



 



\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{velocity_error_1.pdf}
\caption{Velocity Control output during testing }
\label{vel_error_test}
\end{figure}



\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{control_error_1.pdf}
\caption{Lateral Control output during testing }
\label{control_error_test}
\end{figure}

## Adaptation for specific service
Depending on the specific task, various tools which include sensors and/or actuators that are necessary for the service task can be mounted onto the ASV. These sensors and actuators are collectively called as a workload. The ASV modules are customizable to carry the workload and perform the desired service task. The ASV localize itself by using information from map and GPS, thus eliminating the need for installing environmental guidance such as RFID [@7225673] or magnetic rails [@5354591]. For inspection and surveillance purposes, the camera data can be streamed and recorded via the HMI. For cleaning service, the ASV perform its task during its movement. This necessitates that the path planning follows certain objects, such as road pavement or curb. Combined with the method proposed in [@SongZW_IV_2015], the ASV can navigate to track arbitrary distinguishable object boundaries to conduct the cleaning. Nevertheless, thanks to the developed localization, arbitrary path rerouting is also possible. For services that require following certain moving object such as pedestrian [@7139259], the LIDAR and camera sensors could be used to track the object. Subsequently, the tracked object positions will become the destination points for the ASV path planner.

# Conclusion

The ASV has been developed to handle human service tasks. It has generic architecture which is reproducible for other purpose of use. During the development of the ASV, issues had surfaced, and dealt with current approaches. The main function of the ASV is to navigate through set of prior taught waypoints. Precise localization is achieved by combining GPS reading and a prior map generated using LIDAR. This allows adaptation to various human oriented service tasks, with appropriate tooling mounted. The experimental prototype results verified that the ASV is able to execute the path reliably and safely. Currently, the low level controller has been implemented in an embedded microcontroller platform. It is of current ongoing work that other functional modules will be implemented in a compact, low cost and low power compute device, enabling possibilities of efficiently retrofitting compact vehicles or existing service machines into ASVs. This full embedded implementation and more experiments on study to deal with specific services will be conducted and reported in the future.

<!--
That’s all folks!
-->
