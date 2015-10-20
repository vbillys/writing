\thanks{*This work was supported by Autonomous Vehicle Department, Institute for Infocomm Research}

# Introduction

The computational requirements for autonomous driving are very demanding, because autonomous vehicles have to process large amount of online and offline data in order to sense and understand their environment within long range safety distances with high precision, make and perform driving decisions at real-time. Most current autonomous vehicle prototypes around the world employ powerful distributed systems consisting of multiple heterogeneous processors and systems for sensory data processing, sensor fusion, intelligent behavior, and driving control. These types of prototype systems are bulky, costly and power hungry, thus are not suitable for real world deployment. Recently, Audi developed an all-in-one embedded platform code named zFAS which incorporates multiple heterogeneous processors (ECU/MCUs, Application Processors, GPUs, FPGA and a video detection subsystem) on a single tightly integrated board with specialized communication architecture (including Deterministic Ethernet) to suit autonomous driving needs. The computing power equals to the entire electronics architecture of today’s Audi vehicles, yet at a fraction of the size, cost and power consumption. Such a platform is invaluable to eventually bring autonomous driving technologies to the mass market.

This paper aims to describe and propose a generic design of the system architecture based on the chosen compute components and the characteristics / requirements of sensor fusion and autonomous driving applications, integration, test and characterization for a class of small service vehicle.


# Architecture
Components of a typical autonomous or assistive driving system could be summarized in Fig. \ref{fig_1}. The system heavily depends on various types of sensory data and sensors of the same type are synced with one another through data sync modules. To make sense of sensor data, raw data processing is required for different types of available data. Overlapping field of views from different sensors could be combined and the processing result is fused through a Low Level Fusion module. This module also does some filtering to eliminate local noises and thus, make the information more robust to small disturbances. The output objects and localization data from low level fusion modules are then sent to a High Level Fusion Module whereby all objects and filtered sensor information are fused together to form a world model.

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{fig_ex}
\caption{Block Diagram of an autonomous vehicle driving system.}
\label{fig_1}
\end{figure}


## Middleware
[XME? Please fill in]

# Components


## Drive-By-Wire


## Control


## Path Planning

\citeauthor{SongZW_IV_2015}

## Mapping
Procedure of mapping includes:

1. Point cloud assembly
2. Point cloud registration
3. Extracting useful information

### Point cloud assembly


### Point cloud registration


### Extracting useful information

* Road curb
* Traffic signs/lights
* Building features

#### Road curb

#### Traffic signs/lights
\begin{table}[!t]\renewcommand{\arraystretch}{1.3}\caption{A Simple Example Table, IEEE standard format}\label{table_example}\centering\begin{tabular}{c||c}\hline\bfseries First & \bfseries Next\\\hline\hline1.0 & 2.0\\\hline\end{tabular}\end{table}

#### Building features

The *Gamma function* satisfying $\Gamma(n) = (n-1)!\quad\forall n\in\mathbb N$ is via the euler integral$$\Gamma(z) = \int_0^\infty t^{z-1}e^{-t}dt\,.$$

## Localization
The localization is done by modeling the problem using **Gamma function** satisfying $\Gamma(n) = (n-1)!\quad\forall n\in\mathbb N$ is via the euler integral:
\begin{equation}
\label{eqn_example}
x = \sum\limits_{i=0}^{z} 2^{i}Q
\end{equation}

# Safety Consideration

$\mathbf{V}_1 \times \mathbf{V}_2 =  \begin{vmatrix}\mathbf{i} & \mathbf{j} & \mathbf{k} \\\frac{\partial X}{\partial u} &  \frac{\partial Y}{\partial u} & 0 \\\frac{\partial X}{\partial v} & \frac{\partial Y}{\partial v} & 0\end{vmatrix} \label{eqn_md2}$

# Experimentation
Lorem ipsum dolor sit amet, consectetuer adipiscing elit. Pellentesque malesuada ante eu quam. Vivamus tristique. Vivamus nec ante eget felis egestas scelerisque.

## Captions and Reference

## Citations

@Corley-etal_2011[@Corley-etal_2012]

\citeauthor{Corley-etal_2011}

\citetext{\citeyear{Corley-etal_2011}; \citealp{Corley-etal_2012}}

\citep{Corley-etal_2011}

\citet{Corley-etal_2012}

butts [@Corley-etal_2011]

![Cool figure](example) {.two}

# Conclusion

yes, @Corley-etal_2012 confirmed butts
