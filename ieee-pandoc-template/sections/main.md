# Introduction

Recent advancements in robotics technology have brought great benefit to autonomous vehicle research. Since the Defense Advanced Research Projects Agency (DARPA) 2003 Grand Challenge [@ventures2006stanley] , many research groups have focused their effort to solving problems related to autonomous vehicle navigation. As laboratories and research centers bringing the autonomous vehicle technologies to real world applications, acceptance of fully unmanned vehicles will still be a challenge. However, with current progress it can be shown that for a specific problem domain, in this case service vehicles, the autonomous vehicle can be utilized for automation of human service oriented tasks, such as automated road cleaning and grass sucking machine. The Autonomous Vehicle Department at the Institute for Infocomm Research has developed an Autonomous Service Vehicle (ASV) as a prototype to demonstrate autonomous vehicle capability tasked for a service vehicle.

The computational requirements for autonomous driving are very demanding, because autonomous vehicles have to process large amount of online and offline data in order to sense and understand their environment within long range safety distances with high precision, make and perform driving decisions at real-time. Most current autonomous vehicle prototypes around the world employ powerful distributed systems consisting of multiple heterogeneous processors and systems for sensory data processing, sensor fusion, intelligent behavior, and driving control. These types of prototype systems are bulky, costly and power hungry, thus are not suitable for real world deployment. Recently, Audi developed an all-in-one embedded platform code named zFAS which incorporates multiple heterogeneous processors (ECU/MCUs, Application Processors, GPUs, FPGA and a video detection subsystem) on a single tightly integrated board with specialized communication architecture (including Deterministic Ethernet) to suit autonomous driving needs. The computing power equals to the entire electronics architecture of today’s Audi vehicles, yet at a fraction of the size, cost and power consumption. Such a platform is invaluable to eventually bring autonomous driving technologies to the mass market.

This paper contribution is threefold. First, to describe and propose a generic design of the system architecture based on necessary components for the ASV applications, and its integration. Second, we propose the ASV architecture to be adopted into generic service plaform for both research and application purposes. Although, the implementation of the ASV presented here should be specific to certain vendors, the components required to  build an ASV are generic and should be reproducible with similar effort. Lastly, we presented discussion on issues surfaced during tests and characterization of the ASV performance.

## Problem Formulation




# Architecture

Components of a typical autonomous or assistive driving system could be summarized in Fig. \ref{fig_1}. The system heavily depends on various types of sensory data and sensors of the same type are synced with one another through data sync modules. To make sense of sensor data, raw data processing is required for different types of available data. Overlapping field of views from different sensors could be combined and the processing result is fused through a Low Level Fusion module. This module also does some filtering to eliminate local noises and thus, make the information more robust to small disturbances. The output objects and localization data from low level fusion modules are then sent to a High Level Fusion Module whereby all objects and filtered sensor information are fused together to form a world model.

<!--
\begin{algorithm}
 \KwData{this text}
 \KwResult{how to write algorithm with \LaTeX2e }
 initialization\;
 \While{not at end of this document}{
  read current\;
  \eIf{understand}{
   go to next section\;
   current section becomes this one\;
   }{
   go back to the beginning of current section\;
  }
 }
 \caption{How to write algorithms}
\end{algorithm}
-->

<!--
\begin{algorithm}
\caption{CH election algorithm}
\label{CHalgorithm}
\begin{algorithmic}[1]
\Procedure{CH\textendash Election}{}
\For{each node $i$ \Pisymbol{psy}{206} $N$ }
\State Broadcast HELLO message to its neighbor
\State let $k$ \Pisymbol{psy}{206} $N1$ ($i$) U {$i$} be s.t
\State QOS($k$) = max {QOS($j$) \textbar $j$ \Pisymbol{psy}{206} $N1$($i$)  U $i$}
\State MPRSet($i$) = $k$
\EndFor
\EndProcedure
\end{algorithmic}
\end{algorithm}
-->

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{HighLevel.pdf}
\caption{Block Diagram of an autonomous vehicle driving system.}
\label{fig_1}
\end{figure}


## Middleware

Middleware refers to the software component that connects the various modules across network distributed system. It makes communication transparent to connected modules by providing reliability, consistency, security,   and privacy capabilities. In our architecture CHROMOSOME[@CHROMOSOME] a Message-oriented middleware is used. CHROMOSOME implements publish-subscribe message paradigm, which also provides more guarantees with respect to the behaviour of distributed applications. [@buckl2014chromosome] The other  alternative middlewares are Robotics Operating System (ROS) [@quigley2009ros] and  OpenRTM-aist [@ando2005rt] which are commonly used in the field of Robotics. 

## Localization


Fig \ref{fig_2} shows a control error during simulation. A Stanley controller can be used for different type of course, such as parking, low speed, and medium travels.

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{fig_ex.pdf}
\caption{Example of matplotlib graph (vector graphic in the paper).}
\label{fig_2}
\end{figure}

<!--
Path planning done by \citeauthor{SongZW_IV_2015} has revealed the fact that small service vehicle can be robustly navigate the path as long as the localization gives its correct position. For more examples, readers are encouraged to read [@Corley-etal_2011]. In addition, @Corley-etal_2012 confirmed that localization can be affected by surrounding dynamic environment changes such as parked cars, and moving pedestrians. @Aalst-etal_2004 proposed a solution to this, but later the work has been verified not working in [@Abadi-etal_2008;@Abebe-etal_2009]. For more references on this problem, suggested readings are [@Ackerman-Halverson_1998;@Agrawal-etal_1998;@Ali-etal_2012;@Alipour-etal_2013].
-->

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

..Or, using numbering and some examples of referencing them given above.


## Perception

(LIDAR) based, but camera just for logging and teleop

### Obstacle Detection



## Path Planning

Path planning module will help to generate reference trajectory path for autonomous vehicle to follow from a start point to a goal point. The ASV's path planning is implemented in two stages: Global Path Planning and Local Path Planning. 

### Global Path Planning

Global path planning aims to generate the global waypoints to reach destination from origin point. Global path planning can be achieved by notable path planning algorithm such $A^{*}$, $D^{*}$, Rapidly exploring random map, probabilistic roadmap. In our case for teach and repeat solution, global path planning is generated by manual drive and record the UTM coordinates for path driven.  Recorded path are later downsampled by post processing algorithm \ref{eqn_global_path} for saving memory and path optimization. During post processing duplicate UTM points (x,y), straight line points, points within vehicle turning radius are downsampled.  For example if original recorded waypoints could contain ~35000 UTM points (x,y), after post processing the global waypoints will contain ~100 UTM points.  

\begin{algorithm}
       \scriptsize
       \algsetup{linenosize=\scriptsize}
       \caption{Generate Global Waypoints}
       \label{alg_global_waypoints}
       \begin{algorithmic}[1]
           \REQUIRE minLen, minRad, recordedPoint
           	\STATE initialize points[], globalPoints[]
           	\STATE totalGlobalPoints = 0
           	
           	\FORALL{$points$ in $recordedPoint$}
           		\IF{$totalGlobalPoints$ == 0}
           			\STATE append $points$ to $globalPoints$
           			\STATE $totalGlobalPoints++$
           		\ELSE
           			\IF{(distance between $points$  and $globalPoints[totalGlobalPoints - 1]$) $\leq$ $minLen$}
           				\STATE Continue
           			\ELSIF{($\arctan$ between $points$ and $globalPoints[totalGlobalPoints - 1]$) $\leq$ minRad}
           				\STATE Continue
           			\ELSE
           				\STATE append $points$ to $globalPoints$
           				\STATE $totalGlobalPoints++$
           			\ENDIF
           		\ENDIF
			\ENDFOR
       \end{algorithmic}
\end{algorithm}

<!--
\begin{equation}
\label{eqn_global_path}
define MIN_SAMPLE_LEN
define MIN_SAMPLE_RADIAN
global_points[]
points[]
total_global_points = 0
while points in  recorded_points:
     if total_global_points == 0:
         append points to global_points
         increment total_global_points by one
     else:
         if distance between points and global_points[total_global_points - 1] < MIN_SAMPLE_LEN:
continue
         else if arc tangent between points and global_points[total_global_points - 1] < MIN_SAMPLE_RADIAN:
continue
         else	
                append points to global_points
                increment total_global_points by 1
\end{equation}
-->


### Local Path Planning

Local path planning is also termed as motion planning which aims to generate dynamic reference trajectory with vehicle dynamics constraints such as turning radius, velocity from point A to point B. Global path planning generates reference trajectory from start point to goal point, whereas Local path planning is recursively generated along global path for specified distance with higher sample rate to achieve smoother vehicle navigation. Local path planning helps to deal with dynamic circumstances like avoiding obstacle along planned global path.

In our local path planning module is developed with Dubins-Curve library [@DubinsCurves]  which based Dubins path method [@dubinscurve1957]. Dubins path is one of the simplest geometric method to compute shortest path between two points in  Euclidean plane with curvature (Turning radius) as constraint on the path. Dubins path is suitable nonholonomic wheeled mobile robot or car like autonomous vehicles. Car like wheeled robots or autonomous vehicle will motion such going straight (S), turning left(L) and turning right(R). Thus according to Dubins [@dubinscurve1957] optimal path will be always combinational of {LSL, RSR, RSL, LSR, LRL, RLR} this six possibilities. For details description on Dubins curves and equations, readers are encouraged to read section 15.3.1 in [@lavalle2006planning]
	

<!--
Path planning done by \citeauthor{SongZW_IV_2015} has revealed the fact that small service vehicle can be robustly navigate the path as long as the localization gives its correct position. For more examples, readers are encouraged to read [@Corley-etal_2011]. In addition, @Corley-etal_2012 confirmed that localization can be affected by surrounding dynamic environment changes such as parked cars, and moving pedestrians. @Aalst-etal_2004 proposed a solution to this, but later the work has been verified not working in [@Abadi-etal_2008;@Abebe-etal_2009]. For more references on this problem, suggested readings are [@Ackerman-Halverson_1998;@Agrawal-etal_1998;@Ali-etal_2012;@Alipour-etal_2013].
-->

<!--
### Obstacle Avoidance???
-->


## Control    

Vehicle control or autonomous navigation is  one of the key component for autonomous vehicle. Vehicle control performs necessary action to follow the intended path generated by path planner for smoother navigation towards destination point. The gas, brake control for longitudinal direction will be handled by velocity control and the steering control for turning across lateral direction will handled by lateral control module.

### Velocity control    

Velocity control will estimate the gas, brake motor position in the drive by wire system for maintaining the desired velocity and braking for the vehicle. The desired velocity of the vehicle is determined with the modules input such as path planning, obstacle detection and health monitoring system. 
For autonomous lateral control the fixed constant velocity will not be a wise choice, since the navigation path is not always straight line of travel. The path may consists of various turns and unexpected circumstances like obstacles. For smoother path following and avoid jerkiness of vehicle, speed profiling have to done based on path curvatures and speed limits. Based on the data collected by manual driving, the speed is calculated for various path curvatures (turns). During the local path generation, the curvature of path is calculated and desired speed will be assigned to each curvatures across the path. To avoid rapid speed change and to make velocity transition smooth from straight line to curvature areas velocity averaging is performed in the direction of end to path start.
Apart from speed determined by speed profiler across the path, in event of obstacle detection vehicle will also perform action like slow down or stops based on obstacle detected distance. In the event of certain module error or network communication  failure, health monitoring system will perform stop action based on severity of error.
To achieve desired velocity, the controller need to calculate the desired gas/brake position of the motor. Thus one of the frequently used controller technique closed loop PID controller is implemented. The closed loop PID controller helps in noise rejection and robustness of control with design flexibility. 
ased on desired velocity, the controller will calculate the motor position and passes to drive by wire (DBW) system. Drive by wire system will convert controller command to control signal and passes to the motor. The wheel encoder will help to measure the speed precisely and feedback to the controller for the necessary action. The block diagram of the velocity control implementation is shown in Fig \ref{fig_vel_control}. The governing control law is given in the following calculation: 

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
The heading error $\theta_{e}$ is calculated by difference in vehicle heading $\theta$ and reference heading $\theta_{r}$ of path at $(R_{x}, R_{y})$. This term error directly apply to steering angle $\delta$ with some steering angle limitation.
\begin{equation}
\label{eqn_heading_error}
\theta_{e} = \theta - \theta_{r}
\end{equation}
When the cross track error $C_{e}$ is non-zero, it adjusts $\delta$ such that the intended trajectory intersects the reference path tangent from $(R_{x}, R_{y})$ at velocity in time $V_{t}$ units from the front axle  center point $(P_{x}, P_{y})$. The resulting steering control law by combining two terms are shown in (\ref{eqn_stanley_equation}). The figure \ref{fig_stanley} illustrates the geometric relationship of control parameters.

\begin{equation}
\label{eqn_stanley_equation}
\delta(t) = k_{h}\theta_{e} + tan^{^{-1}}\left (\frac{k_{c}C_{e}(t)}{V_{x}(t)}  \right )
\end{equation}

Fig ?? shows a control error during simulation. A Stanley controller can be used for different type of course, such as parking, low speed, and medium travels.

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{stanley_2.pdf}
\caption{Stanley controller geometry}
\label{fig_stanley}
\end{figure}


## Health Monitoring  


Safety is very important in a critical mission project like autonomous service vehicle. Not only that robust operation is required, but also human safety must be prioritized in the event of malfunctions or catastrophic failures. Health monitoring system is based on publish-subscribe model. This allows both centralized and distributed monitoring model. In the initialization stage, each ASV functional module component subscribes to a health topic proceeding completion of self-diagnosis. The health topic is provided by the Central Health Monitor (CHM). In the operational stage, modules are to send unique description and event identification to the CHM. 

The CHM is responsible for reporting any important event to the user, log the event, and take suitable action based on severity of the detected failure or error. In addition, each module has their own reporting module, but limited only to each module scope of operation. The severity levels and the corresponding actions can be summarized in Table \ref{table_risk_hm}. 

\begin{table}[!t]
\renewcommand{\arraystretch}{1.3}
\caption{Severity levels in health monitoring module}
\label{table_risk_hm}
\centering
\begin{tabular}{c||c}
\hline\bfseries Severity & \bfseries Actions\\
\hline
\hline None & Normal Operation. Status Logging\\
\hline
\hline Warn & Display warning on user interface. Status logging. Health monitor observes.\\
\hline
\hline Abort & Vehicle aborts mission, plans and stops at a safe zone. Status logging and displaying on user interface. Health monitor observes.\\
\hline
\hline Emergency Stop & Vehicle aborts mission immediately. Status logging and displaying on user interface. All modules restarts and perform self-diagnosis.\\
\end{tabular}
\end{table}

## Human Machine Interface
Web based Human Machine Interface (HMI) is developed to monitor and control vehicle remotely. 




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
(GPS IMU etc)

#### 2D LIDAR

#### 3D LIDAR


#### Camera


### Safety System


## Testing Results and Evaluation










\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{map_overlay.pdf}
\caption{Recorded Waypoint to travel}
\label{map_waypoint}
\end{figure}


\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{vehicle_travelled_label.pdf}
\caption{Vehicle travelled path overview}
\label{veh_path_overview}
\end{figure}

Fig. 2. Example of matplotlib graph (vector graphic in the paper)

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{vehicle_travelled_crop.pdf}
\caption{Vehicle avoidance path }
\label{veh_avoid_close}
\end{figure}

# Conclusion

That’s all folks!

