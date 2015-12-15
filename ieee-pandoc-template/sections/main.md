\thanks{*This work was supported by Autonomous Vehicle Department, Institute for Infocomm Research}

# Introduction

The computational requirements for autonomous driving are very demanding, because autonomous vehicles have to process large amount of online and offline data in order to sense and understand their environment within long range safety distances with high precision, make and perform driving decisions at real-time. Most current autonomous vehicle prototypes around the world employ powerful distributed systems consisting of multiple heterogeneous processors and systems for sensory data processing, sensor fusion, intelligent behavior, and driving control. These types of prototype systems are bulky, costly and power hungry, thus are not suitable for real world deployment. Recently, Audi developed an all-in-one embedded platform code named zFAS which incorporates multiple heterogeneous processors (ECU/MCUs, Application Processors, GPUs, FPGA and a video detection subsystem) on a single tightly integrated board with specialized communication architecture (including Deterministic Ethernet) to suit autonomous driving needs. The computing power equals to the entire electronics architecture of today’s Audi vehicles, yet at a fraction of the size, cost and power consumption. Such a platform is invaluable to eventually bring autonomous driving technologies to the mass market.

This paper aims to describe and propose a generic design of the system architecture based on the chosen compute components and the characteristics / requirements of sensor fusion and autonomous driving applications, integration, test and characterization for a class of small service vehicle.


# Architecture

Components of a typical autonomous or assistive driving system could be summarized in Fig. \ref{fig_1}. The system heavily depends on various types of sensory data and sensors of the same type are synced with one another through data sync modules. To make sense of sensor data, raw data processing is required for different types of available data. Overlapping field of views from different sensors could be combined and the processing result is fused through a Low Level Fusion module. This module also does some filtering to eliminate local noises and thus, make the information more robust to small disturbances. The output objects and localization data from low level fusion modules are then sent to a High Level Fusion Module whereby all objects and filtered sensor information are fused together to form a world model.

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{HighLevel.pdf}
\caption{Block Diagram of an autonomous vehicle driving system.}
\label{fig_1}
\end{figure}


## Middleware

Middleware refers to the software component that connects the various modules across network distributed system. It makes communication transparent to connected modules by providing reliability, consistency, security,   and privacy capabilities. In our architecture CHROMOSOME[@CHROMOSOME] a Message-oriented middleware is used. CHROMOSOME implements publish-subscribe message paradigm, which also provides more guarantees with respect to the behaviour of distributed applications. [@buckl2014chromosome] The other  alternative middlewares are Robotics Operating System (ROS) [@quigley2009ros] and  OpenRTM-aist [@ando2005rt] which are commonly used in the field of Robotics. 

# Components


## Drive-By-Wire


## Control

Fig \ref{fig_2} shows a control error during simulation. A Stanley controller can be used for different type of course, such as parking, low speed, and medium travels.

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{fig_ex.pdf}
\caption{Example of matplotlib graph (vector graphic in the paper).}
\label{fig_2}
\end{figure}

## Path Planning

Path planning done by \citeauthor{SongZW_IV_2015} has revealed the fact that small service vehicle can be robustly navigate the path as long as the localization gives its correct position. For more examples, readers are encouraged to read [@Corley-etal_2011]. In addition, @Corley-etal_2012 confirmed that localization can be affected by surrounding dynamic environment changes such as parked cars, and moving pedestrians. @Aalst-etal_2004 proposed a solution to this, but later the work has been verified not working in [@Abadi-etal_2008;@Abebe-etal_2009]. For more references on this problem, suggested readings are [@Ackerman-Halverson_1998;@Agrawal-etal_1998;@Ali-etal_2012;@Alipour-etal_2013].

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



## Localization

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

## Path Planning

Citations examples...

Path planning done by \citeauthor{SongZW_IV_2015} has revealed the fact that small service vehicle can be robustly navigate the path as long as the localization gives its correct position. For more examples, readers are encouraged to read [@Corley-etal_2011]. In addition, @Corley-etal_2012 confirmed that localization can be affected by surrounding dynamic environment changes such as parked cars, and moving pedestrians. @Aalst-etal_2004 proposed a solution to this, but later the work has been verified not working in [@Abadi-etal_2008;@Abebe-etal_2009]. For more references on this problem, suggested readings are [@Ackerman-Halverson_1998;@Agrawal-etal_1998;@Ali-etal_2012;@Alipour-etal_2013].


## Control    

Vehicle control or autonomous navigation is  one of the key component for autonomous vehicle. Vehicle control performs necessary action to follow the intended path generated by path planner for smoother navigation towards destination point. The gas, brake control for longitudinal direction will be handled by velocity control and the steering control for turning across lateral direction will handled by lateral control module.

### Velocity control    

Velocity control will estimate the gas, brake motor position in the drive by wire system for maintaining the desired velocity and braking for the vehicle. The desired velocity of the vehicle is determined with the modules input such as path planning, obstacle detection and health monitoring system. 
For autonomous lateral control the fixed constant velocity will not be a wise choice, since the navigation path is not always straight line of travel. The path may consists of various turns and unexpected circumstances like obstacles. For smoother path following and avoid jerkiness of vehicle, speed profiling have to done based on path curvatures and speed limits. Based on the data collected by manual driving, the speed is calculated for various path curvatures (turns). During the local path generation, the curvature of path is calculated and desired speed will be assigned to each curvatures across the path. To avoid rapid speed change and to make velocity transition smooth from straight line to curvature areas velocity averaging is performed in the direction of end to path start.
Apart from speed determined by speed profiler across the path, in event of obstacle detection vehicle will also perform action like slow down or stops based on obstacle detected distance. In the event of certain module error or network communication  failure, health monitoring system will perform stop action based on severity of error.
To achieve desired velocity, the controller need to calculate the desired gas/brake position of the motor. Thus one of the frequently used controller technique closed loop PID controller is implemented. The closed loop PID controller helps in noise rejection and robustness of control with design flexibility. 
Based on desired velocity, the controller will calculate the motor position and passes to drive by wire (DBW) system. Drive by wire system will convert controller command to control signal and passes to the motor. The wheel encoder will help to measure the speed precisely and feedback to the controller for the necessary action. The block diagram of the velocity control implementation is shown in Fig \ref{fig_vel_control}. 



\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{pid_velocity.pdf}
\caption{Velocity control based on closed loop PID.}
\label{fig_vel_control}
\end{figure}


### Lateral Control    
Lateral control aims to control the steering angle of the vehicle based on trajectory generated by path planner. In our study we have developed lateral controller based on Stanley’s [@ventures2006stanley] steering controller. Stanley steering controller is one of the popular geometric path tracking approach used by Stanford university’s autonomous vehicle during DARPA grand challenge 2006. 
The closed-loop geometric tracking controller helps vehicle to follow the desired path generated by path planning. The controller is based on non-linear cross track error $C_{e}$ and heading error $\theta_{e}$. Cross track error is the lateral distance between front axle center point (P_{x}, P_{y}) and closest reference point (R_{x}, R_{y}) for which exponential can be shown[@ventures2006stanley]. This term helps the vehicle to steer towards the path to yield $C_{e}$ as zero when its following the path. 
The heading error $\theta_{e}$ is calculated by difference in vehicle heading $\theta$ and reference heading $\theta_{r}$ of path at $(R_{x}, R_{y})$. This term error directly apply to steering angle $\delta$ with some steering angle limitation.
\begin{equation}
\label{eqn_heading_error}
\theta_{e} = \theta - \theta_{r}
\caption{Heading error}
\end{equation}
When the cross track error $C_{e}$ is non-zero, it adjusts $\delta$ such that the intended trajectory intersects the reference path tangent from $\(R_{x}, R_{y})$ at velocity in time $V_{t}$ units from the front axle  center point $\(P_{x}, P_{y})$. The resulting steering control law by combining two terms are shown in (\ref{eqn_stanley_equation}). The figure \ref{fig_stanley} illustrates the geometric relationship of control parameters.

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


# Health Monitoring  


Safety is very important in a critical mission project like autonomous service vehicle. Not only that robust operation is required, but also human safety must be prioritized in the system logic. Some components in the system that needs attention are listed in Table \ref{table_risk_hm}.

\begin{table}[!t]
\renewcommand{\arraystretch}{1.3}
\caption{Risk in safety assessment for health monitoring module}
\label{table_risk_hm}
\centering
\begin{tabular}{c||c}
\hline\bfseries Component & \bfseries Risk\\
\hline
\hline Drive-By-Wire & Malfunction of motors\\
\hline
\hline GPS & GPS denied\\
\hline
\hline Camera & Cable loose\\
\hline
\end{tabular}
\end{table}


# Experimentation

Fig. \ref{coms1_bare} shows the first vehicle platform that we first retrofitted for the autonomous vehicle project.

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{coms1_bare}
\caption{Experimentation of building vehicle platform.}
\label{coms1_bare}
\end{figure}

## Sensors

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


# Conclusion

That’s all folks!

