# Introduction

Recent advancements in robotics technology have brought great benefit to autonomous vehicle research. Since the Defense Advanced Research Projects Agency (DARPA) 2003 Grand Challenge [@ventures2006stanley], many research groups have focused their effort to solving problems related to autonomous vehicle navigation. As laboratories and research centers bringing the autonomous vehicle technologies to real world applications, acceptance of fully unmanned vehicles will still be a challenge. However, with current progress it can be shown that for a specific problem domain, in this case service vehicles, the autonomous vehicle can be utilized for automation of human service oriented tasks, such as automated road cleaning,  grass cutting and leaves sucking machine. This is supported by the growth of service robotics industry in the recent years [@SVR_Nov_15].  \citeauthor{7225673} [@7225673]  introduced mobile service robot for electrical power plant inspection. This in turn could reduce or replace human involvement in manual inspection process. \citeauthor{6629559} [@6629559] presented an autonomous driving car for research purposes that includes features that allows for urban road navigation. Similarly, \citeauthor{7225765} [@7225765] introduced a more advanced autonomous driving platform consisting of high performance cameras, LIDARs, and other navigation sensors. While these platforms are very attractive in term of their capability of handling big streams of sensors data, the demanding computing power renders them unsuitable for small service vehicle. The Autonomous Service Vehicle (ASV) proposed in this paper is developed as a prototype to demonstrate autonomous vehicle capability tasked for a service vehicle.

Autonomous platforms are usually retrofitted from manually driven vehicles. This approach is commonly used because it is the most viable and affordable way to install a Drive-By-Wire (DBW) system into existing vehicle rather than to entirely build a new one. It was also studied that automation driver’s task has positive impact to improving driving safety [@stanton1996fly]. Various effort to assist drivers has lead to development of ADAS system [@6232125;@7225760]. A DBW enables computerized control of the vehicle by means of replacing human drivers with actuators, which are usually driven by electrical motors [@1035218]. The ASV presented in this paper is retrofitted from a compact electric vehicle [@SongZW_IV_2015], and thus we studied the feasibility of converting this class of vehicles into service vehicles. 

This paper contribution is threefold. First, to describe and propose a generic design of the system architecture based on necessary components for the ASV applications. Second, to propose the ASV architecture to be adopted into generic service platform for both research and application purposes. Although, the implementation of the ASV presented here are subject to certain sources, the components are generic and should be reproducible with similar effort. Lastly, to present discussion on issues surfaced during tests and evaluation of the ASV navigation in urban road environment.

# Architecture

The main service types for ASV are related to defense, agriculture and inspection, logistics and medical applications [@SVR_Nov_15]. As a platform, ASV is required to navigate through environment to complete the assigned service tasks. The approach taken is to modify current vehicle to enable automated control by means of processing sensors data. In this paper, the ASV components are designed to complete a case of urban road navigation. Components of the proposed ASV can be summarized in Fig. \ref{fig_1}. The system heavily depends on various types of sensory data and sensors of the same type are synced with one another through data sync modules. To make sense of sensor data, raw data processing is required for different types of available data. Overlapping field of views from different sensors could be combined and the processing result is fused through a Low Level Fusion module. This module also does some filtering to eliminate local noises and thus, make the information more robust to small disturbances. The output objects and localization data from low level fusion modules are then sent to a High Level Fusion Module whereby all objects and filtered sensor information are fused together to form a world model.






\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{HighLevel.pdf}
\caption{Block Diagram of an autonomous vehicle driving system.}
\label{fig_1}
\end{figure}



## Middleware
Middleware refers to the software component that connects the various modules across network of distributed system. It makes communication transparent to connected modules by providing reliability, consistency, security,   and privacy capabilities. In our architecture CHROMOSOME (abbreviated as XME) [@CHROMOSOME], a message-oriented middleware is used. XME implements the publish-subscribe message paradigm, which provides guarantees with respect to the behaviour of distributed applications [@buckl2014chromosome]. The other  alternative middlewares are Robotics Operating System (ROS) [@quigley2009ros] and  OpenRTM-aist [@ando2005rt] which are commonly used in the field of robotics. 


## Localization



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



<!--
## Perception

(LIDAR) based, but camera just for logging and teleop
-->

## Obstacle Detection
During operation of the ASV, obstacle obstructing or near the vehicle should be considered. Thus, by using a laser range sensor model for detection, a set $L$ of $n$ points which each has a range $r_{i}$ and a angle $\alpha_{i}$, measured from laser beams $L=\left\{(r_{1},\alpha_{1}),...,(r_{n},\alpha_{n})\right\}$ are clustered to extract objects [@Juric-Kavelj2008_407]. A Gaussian smoothing is applied to remove spurious noise before processing. Two subsequent points belong to one cluster satisfies the distance threshold  $d(r_{i},r_{i+1}) \leq D_{0}+D_{1}min(r_{i},r_{i+1})$, where $D_{0}$ is the accuracy of the range data. $D_{1} = \frac{3}{2}(\alpha_{i+1}-\alpha_{i})$ is a constant that determine maximum change allowed. Point that break the threshold will be the start of another cluster. Processed laser scanner is represented by a set of clusters $\left\{ C_{1},...,C_{m} \right\}$. Each cluster consists of tuples $({}^{C}r_{i},{}^{C}\alpha_{i},{}^{C}x_{i},{}^{C}y_{i})$ where ${}^{C}x_{i}$, ${}^{C}y_{i}$ are the Cartesian coordinate representations.

Each cluster is a potentially detected object. Based on the lateral proximity of each cluster, object boundaries are found by determining if each cluster satisfies $w_{min} \leq d(y_{min},y_{max})  \leq w_{max}$, where $w_{max}$ and $w_{min}$ depend on widths of object of interest. The detected obstacles are sent the path planner to aid in calculation of the navigation course. Moreover, obstacle detection improves false recognition removal of map objects for localization [@5164269]. The cluster points of detected objects are used to determine areas of where measured points should be excluded from the localization and mapping process.



## Path Planning

Path planning module will help to generate reference trajectory path for autonomous vehicle to follow from a start to a destination point. The ASV path planning is implemented in two stages: Global Path Planning and Local Path Planning. 

### Global Path Planning
Global path planning aims to generate the global waypoints to reach destination from origin point. Global path planning can be achieved by notable path planning algorithm such $A^{*}$, $D^{*}$, Rapidly exploring random map, probabilistic roadmap. In our case for teach and repeat solution, global path planning is generated by manual drive and record the UTM coordinates for path driven.  Recorded path are later downsampled by post processing Alg. \ref{alg_global_waypoints} for saving memory and path optimization. During post processing duplicate UTM points, straight line points, points within vehicle turning radius are downsampled.  For a typical example, if original recorded waypoints could contain ~35000 UTM points, after post processing the global waypoints will contain ~100 UTM points.  

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
Our Local Path Planning module is developed with Dubins-Curve [@DubinsCurves] library  which is based Dubins [@dubinscurve1957@]path method. Dubins path is one of the simplest geometric methods to compute the shortest path between two points in the Euclidean plane with turning radius as constraint. Dubins path is suitable for nonholonomic wheeled mobile robot or car like autonomous vehicles. Dubins path is built on top of the assumption that non-holonomic vehicle has a typical motion of going straight (S), turning left(L) and turning right(R). According to Dubins [@dubinscurve1957@]optimal path will be always one of the six possible motion sequences: {LSL, RSR, RSL, LSR, LRL, RLR}. For details on Dubins method, readers can refer to section 15.3.1 in [@lavalle2006planning].
Local Path Planning Alg. \ref{alg_local_waypoints} is executed at regular interval when the AVS has travelled a certain distance or any obstacle is detected along the path. In the event of obstacle obstructing the global path, the local path will be shifted to the left or right side of obstacle boundary based on environmental factors for safer avoiding. An example of the Local Path Planning avoidance is shown on Fig. \ref{veh_avoid_close}.


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
Vehicle control or autonomous navigation is  one of the key component for autonomous vehicle. Vehicle control performs necessary action to follow the intended path generated by path planner for smooth navigation towards destination point. The gas and brake control for longitudinal direction will be handled by the velocity control module and the steering control for turning across lateral direction will handled by the lateral control module.

### Velocity control    
Velocity control will estimate the gas, brake motor position in the drive by wire system for maintaining the desired velocity and braking for the vehicle. The desired velocity of the vehicle is determined with the modules input such as path planning, obstacle detection and health monitoring system. 
For autonomous lateral control the fixed constant velocity will not be a wise choice, since the navigation path is not always straight line of travel. The path may consists of various turns and unexpected circumstances like obstacles. For smoother path following and avoid jerkiness of vehicle, speed profiling have to done based on path curvatures and speed limits. Based on the data collected by manual driving, the speed is calculated for various path curvatures. During the local path generation, the curvature of path is calculated and desired speed will be assigned to each curvatures across the path. To avoid rapid speed change and to make velocity transition smooth from straight line to curvature areas velocity averaging is performed in the direction of end to path start.
Apart from speed determined by speed profiler across the path, in event of obstacle detection vehicle will also perform action like slow down or stops based on the obstacle proximity to vehicle. In the event of certain module error or network communication  failure, health monitoring system will adjust the speed based on determined severity.

To achieve desired velocity, a closed loop Proportional-Integral-Derivative (PID) controller is implemented to compute the desired gas/brake position of the motor. The closed loop PID controller helps in noise rejection and robustness of control with design flexibility. 
Based on desired velocity, the controller will calculate the motor position and passes to drive by wire (DBW) system. The wheel encoder measures the speed precisely and provide feedback to the controller. The block diagram of the velocity control implementation is shown in Fig \ref{fig_vel_control}. The governing control law of the PID controller is given in the following:

\begin{equation}
\label{eqn_pid}
GB = k_p{E} + k_{i}\int_{0}^{t}{E}\,{dt} + k_{d}\frac{dE}{dt}
\end{equation}

Where $GB$ is gas-brake position value, $E$ is the velocity error, $dE$ is the differential error, $k_p$ is the proportional gain, $k_i$ is the integral gain, $k_d$ is the differential gain.

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{pid_velocity.pdf}
\caption{Velocity control based on closed loop PID.}
\label{fig_vel_control}
\end{figure}


### Lateral Control

Lateral control aims to control the steering angle of the vehicle based on trajectory generated by path planner. In our study we have developed lateral controller based on Stanley’s [@ventures2006stanley] steering controller. Stanley steering controller is one of the popular geometric path tracking approach used by Stanford university’s autonomous vehicle during DARPA grand challenge 2006. 
The closed-loop geometric tracking controller helps vehicle to follow the desired path generated by path planning. The controller is based on non-linear cross track error $C_{e}$ and heading error $\theta_{e}$. Cross track error is the lateral distance between front axle center point $(P_{x}, P_{y})$ and closest reference point $(R_{x}, R_{y})$ for which exponential can be shown [@ventures2006stanley]. This term helps the vehicle to steer towards the path to yield $C_{e}$ as zero when its following the path. 
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
\includegraphics[width=2.5in]{stanley_2.pdf}
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
Our web based HMI is developed with websocket [@fette2011websocket] protocol implementation for achieving low-latency, long-running  bidirectional communication. For security concern, HMI can be accessed only from intranet with unique private IP assigned for the ASV and user need to authenticate with username and password. Any HTML5 supported browser can be used to access the HMI. The HMI consist of dashboard, system interface and settings interface. They individually has different scope of information and options.
For example, Fig. \ref{fig_hmi} shows the HMI dashboard. The dashboard is essential for vehicle monitoring information such as precise ASV position on the map, vehicle speed, battery level and system log for displaying critical error logs and network status. The system interface is used to manage individual modules. It has options to starting, to stop and to view error logs. The settings interface is used to modify ASV parameters such as maximum speed, braking distance, velocity control and lateral control gain.




\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{hmi.pdf}
\caption{HMI dashboard interface}
\label{fig_hmi}
\end{figure}


# Experimentation

Fig. \ref{coms1_bare} shows the first vehicle platform that we first retrofitted for the autonomous vehicle project.

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{coms1_bare}
\caption{Experimentation of building vehicle platform.}
\label{coms1_bare}
\end{figure}

## Setup

Placement of the sensors on the autonomous vehicle can be seen in Fig. \ref{ipe_ex}, and their connection to the system can be summarized in Fig. \ref{dia_ex}.


\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{ipe_ex.pdf}
\caption{Placement of sensors.}
\label{ipe_ex}
\end{figure}


\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{dia_ex.pdf}
\caption{Diagram of sensors connection.}
\label{dia_ex}
\end{figure}



### Vehicle


### Drive-By-Wire


### Power System	


### Sensors

#### Odometry
<!--
Odometry is a method to fuse the motion sensors data to obtain the position of any mobile robots. 
-->

Odometry position is estimated via accumulating the motion sensors data over time, thus odometry position is usually best for shorter-term. Pure odometry position is not recommended for  long-term, since the accumulation of motion sensor error will cause a drift in the actual position over time. To obtain accurate position for longer-term, odometry will be fused with either gps or lidar or vision based localization. The simple odometry position is derived from the equation \ref{eqn_simple_odometry}.
In our experiment, we used following two motion sensor:
* Wheel motion sensor (Kistler Wheel Pulse Transducers) - Its helps to calculate vehicle travelled distance precisely from wheel motion sensor counts.
* Inertial Measurement Unit (Xsens MTi-G-700 GPS) - Gyroscope in the IMU unit will provide the roll, pitch and yaw attributes at higher frequency data rate. 
\begin{equation}
\begin{split}
\label{eqn_simple_odometry}
\theta = \theta + \Delta\theta \\
x = x + (\Delta v + \cos\theta) \\
y = y + (\Delta v + \sin\theta)
\end{split}
\end{equation}

where $\Delta\theta$ is the yaw rate obtained from IMU, $\theta$ is the heading, $\Delta v$ is distance obtained from wheel motion sensor, x and y are the Euclidean coordinates of the odometry position.

#### 2D LIDAR

#### 3D LIDAR


#### Camera


### Safety System


## Testing Results and Evaluation









<!--
\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{map_overlay.pdf}
\caption{Recorded Waypoint to travel}
\label{map_waypoint}
\end{figure}
-->

<!--
\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{vehicle_travelled_label.pdf}
\caption{Vehicle travelled path overview}
\label{veh_path_overview}
\end{figure}
-->

Fig. 2. Example of matplotlib graph (vector graphic in the paper)

<!--
\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{vehicle_travelled_crop.pdf}
\caption{Vehicle avoidance path }
\label{veh_avoid_close}
\end{figure}
-->

## Adaptation for specific service

Depending on the specific task, various tools which include sensors and/or actuators that are necessary for the service task can be mounted onto the ASV. These sensors and actuators are collectively called as a workload. The ASV modules are customizable to carry the workload and perform the desired service task. The ASV localize itself by using information from map and GPS, thus eliminating the need for installing environmental guidance such as RFID [@7225673] or magnetic rails [@5354591]. For inspection and surveillance purposes, the camera data can be streamed and recorded via the HMI. For cleaning service, the ASV perform its task during its movement. This necessitates that the path planning follows certain objects, such as road pavement or curb. Combined with the method proposed in [@SongZW_IV_2015], the ASV can navigate to track arbitrary distinguishable object boundaries to conduct the cleaning. Nevertheless, thanks to the developed localization, arbitrary path rerouting is also possible. For services that require following certain moving object such as pedestrian [@7139259], the LIDAR and camera sensors could be used to track the object. Subsequently, the tracked object positions will become the destination points for the ASV path planner.


# Conclusion

The ASV has been developed to handle human service tasks. It has generic architecture which is reproducible for other purpose of use. During the development of the ASV, issues had surfaced, and dealt with current approaches. The main function of the ASV is to navigate through set of prior taught waypoints. Precise localization is achieved by combining GPS reading and a prior map generated using LIDAR. This allows adaptation to various human oriented service tasks, with appropriate tooling mounted. The experimental prototype results verified that the ASV is able to execute the path reliably and safely. Currently, the low level controller has been implemented in an embedded microcontroller platform. It is of current ongoing work that other functional modules will be implemented in a compact, low cost and low power compute device, enabling possibilities of efficiently retrofitting compact vehicles or existing service machines into ASVs. This full embedded implementation and more experiments on study to deal with specific services will be conducted and reported in the future.

<!--
That’s all folks!
-->
