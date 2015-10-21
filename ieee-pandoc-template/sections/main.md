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
[XME? Please fill in]

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


# Safety Consideration

Safety is very important in a critical mission project like autonomous service vehicle. Not only that robust operation is required, but also human safety must be prioritized in the system logic. Some components in the system that needs attention are listed in Table \ref{table_example}.


\begin{table}[!t]
\renewcommand{\arraystretch}{1.3}
\caption{A Simple Example Table, IEEE standard format}
\label{table_example}
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

