# Introduction

Mobile service robots, such as robotic butlers, robotic waiters, and robotic pets, are designed to assist people in their daily lives. In these robots, the capability to autonomously detect, track and follow a person has been regarded as an important functionality of the robotic systems. However, until now, it still remains a challenge that endowing this capability to these robots when they operate in human environment.

The approaches of autonomous person following without the person having to wear any active beacon can be classified into two categories: vision based and laser based. Due to the low cost, RGB-D cameras are widely adopted in the vision based approaches as the work presented in [@ref0;@ref1]. However, the minimum distance requirement and the narrow field view of the RGB-D sensors limit the applications for following persons.

The laser based approaches have an advantage in terms of large field of view and robustness on any lighting condition. In \cite{ref3} the map information is used to help tracking both moving and stationary person. However, the usage of this approach will be limited on environments that have been mapped. On the other hand, leg identification has been used for people tracking in an unknown area (e.g. in \cite{ref4} - \cite{ref7}). More recently, \cite{ref7} introduces a framework for outdoor person tracking and following for a smart wheel chair robot by extending a trained legs detector from \cite{ref8}. In [@ref9; @ref10], robotic    behaviors in relation to the speed and distance to the person being    followed have been analyzed. However, the correlation from detecting    legs into tracking a person, and the robustness when multiple people    is around the person being followed has not been explored in these    studies. 

\begin{figure}[!t]
\centering
\includegraphics[width=3.5in]{ISERA.pdf}
\caption{(a) ISERA robot in action of tracking and following a
target person (b) The sensor and interface of ISERA service
robot}\label{fig_isera}
\end{figure}

In this paper, we propose an approach to autonomously classify, track and follow persons in a dynamic environment for a service robot, namely ISERA (shown in Fig. \ref{fig_isera}). Using 2D Lidar and Odometry, our approach does not require prior map information, and benefit from wide field of view and consistent in any lighting condition. In our approach, a constraint based leg detection method is developed to regulate the strictness of the leg identifier as needed. Compared to previous works that solely track the legs, our proposed approach associate the data to form and track a virtual target person. In this way, brief false detections of a leg in crowded environment would not immediately made the person jump to another location, due to physical constraints in relation to the other leg. Two layers of Kalman filter is used to ensure the robustness of the person tracking despite close proximity to another person and temporary line of sight loss to the leg clusters occurred. 

Another notable merit of the proposed approach is that a real time obstacle avoidance algorithm is embedded into the feed-back motion controller of the robot. Hence, experiments show ISERA can follow a target person while avoid obstacles autonomously in a cluttered environment.

The paper is organized as follows. Section II describes the mobile robot system. The people detection, tracking and following approach is detailed in Section III. The experiment and results described in Section VI shows the validity and effectiveness of the proposed approach. Finally, a conclusion is given in Section V.

# System Description

The ISERA robot is designed as a human friendly service robot that could serve as a butler or courier in public area. ISERA is built on the top of a differential drive mobile platform with a maximum speed of $1m/s$. A PC is installed inside the robot for processing and a Hokuyo UTM-30-LX lidar is mounted about $30cm$ from the ground, which corresponds to sensor reading of legs below the knee, around the shank area of average adults.

# Person Following

## Leg Identification

In this paper, we proposes a leg identification that build based on the principle of circle geometric fitting, but instead of giving a crisp output, our algorithm calculates the confidence of a segmented cluster to be considered as a leg. The confidence will then be used in the tracking algorithm to determine the decay rate of the tracker. Another addition is the algorithm could also identify a joint dual leg, where the two legs of a person standing could be very close to each other and seen as one large cluster by the sensor.

### Leg Segmentation

#### Modeling and Evaluation of clusters
To be classified as an arc or convex shape of a leg, the middle point of a cluster must be within a reasonable distance ratio (typically between 0.1-0.7) from the length between the cluster edges. The clusters that pass this filter will be processed further, otherwise it will be discarded. This filtering quickly discards any non-convex shape cluster with minimum computation cost.

Next, the center point of the cluster is approximated as the virtual point in the middle of the cluster leftmost and rightmost edges. For each cluster, the distance between the lidar and the center point of that cluster is denoted as $z$ and will be used as a variable in the analysis. This is to facilitate the effect of distance into the perception data. 

To analyze the single leg clusters, three criterions are used as benchmark. First, the mean error of the inscribe angle variant (IAV) of the cluster [@DBLP:conf/icra/XavierPCRN05]. The second is the error of the standard deviation of the cluster. The third is related to the error in the size of the cluster.

The benchmark criterion is taken from numerous measurement of an adult humanoid model with typical cylindrical legs at varying distance $z$ to the lidar. The data set collected are then interpolated into a function of distance $z$ and kept as a benchmark values.

Then, based on the normalized errors between the desired benchmark and the actual measurement, an equation to formulate the confidence of a cluster to be considered as a leg is derived as,

\begin{equation}
Leg_{c}=max(1-K_II_N-K_SS_N-K_PP_N,0)\,. \label{eq:cost}
\end{equation}
with
\begin{equation}
I_N=(|I_c-I_d(z)|)/{I_d(z)}\,. \label{eq:IAVmean}
\end{equation}
\begin{equation}
S_N=(|S_c-S_d(z)|)/{S_d(z)}\,. \label{eq:IAVStdDev}
\end{equation}
\begin{equation}
P_N=(|P_c-P_d(z)|)/{P_d(z)}\,. \label{eq:points}
\end{equation}

Where $Leg_{c}$, which has a value between $0$ and $1$, is the confidence of cluster $c$ to be considered a leg. $I_c$, $S_c$, and $P_c$ are IAV mean, IAV standard deviation, and number of points that are calculated from sensor data, respectively. $I_d(z)$, $S_d(z)$,and $P_d(z)$ are the desired values of the IAV mean, IAV standard deviation, and number of points in the cluster for distance $z$, respectively. $K_I$, $K_S$, and $K_P$ are weighting factor rule that tuned based on the preferred strictness of the leg segmentation. 

Strict leg segmentation gives the least number of false positives, and could be used when the lidar identifies mostly bare legs. On the other hand, if the circumstance allows us to tolerate more false positives in the leg detection, loose segmentation could be used to detect people that are wearing loose pants. This strictness option would provide situational advantage compared to a rigid data trained learning based methods in other works.

Finally, for clusters that have a confidence that exceed a cut off threshold, the center point and the confidence value are passed on to the leg and person tracking algorithm described in the next subsection.

## Leg and People Tracking

In order to follow certain person, multiple target tracking is developed to estimate positions of both detected legs and people. The estimation is implemented using Kalman Filters (KFs) [@KalmanRudolfEmil] with two different motion model for legs and for people. In general, the state estimation of the legs and people can be modeled by a linear system, $\boldsymbol{\mathrm{x}}_{k+1}=\boldsymbol{A} \boldsymbol{\mathrm{x}}_k + \boldsymbol{\mathrm{w}}_k$. The process noise, $\boldsymbol{\mathrm{w}}_k$ is modelled as Gaussian white noise with covariance matrix $\boldsymbol{Q}$. Fig. \ref{fig_process_diagram} shows the flow of the people following procedure. The input for the legs tracking are the leg detection results in each frame $k$. Furthermore, the input for the people tracking is the leg tracking states, treated as observations. In contrast with our approach, temporary people tracking are created in each update step in [@ref7]. This approach could infer people directly from leg identification, but the movements of the people are not included in the consideration of the filter.  In our method, the KF in the people tracking maintains the state of the tracked people at all times. Finally, the result of the people tracking is transmitted to the pursuit controller (Sec. \ref{pursuit-controller}), so that the robot can follow the person. 

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{process_diagram.pdf}
\caption{Tracking process.}
\label{fig_process_diagram}
\end{figure}

### Legs Tracking
The KF for the legs uses a constant velocity motion model with a pseudo velocity measurement during the KF update steps. At discrete time $k$, the KF maintains a set of leg tracks, ${}^L\boldsymbol{X}_k=\lbrace {}^L\boldsymbol{\mathrm{x}}^1_k, {}^L\boldsymbol{\mathrm{x}}^2_k,\dotsb,{}^L\boldsymbol{\mathrm{x}}^{{}^LN_k}_k \rbrace$, where ${}^LN_k$ is the number of leg track at time $k$. Each leg track has a state estimate ${}^L\boldsymbol{\mathrm{x}}^j_k=[ x \: y \: \dot{x} \: \dot{y} ]^T$ of a target position and velocity in a 2D Cartesian coordinate. During the update step of the Kalman Filter, leg identification are processed using an observation model, ${}^L\boldsymbol{\mathrm{z}}_k={}^L\boldsymbol{H}{}^L\boldsymbol{\mathrm{x}}_k + \boldsymbol{\mathrm{v}}_k$. The observation includes position and velocity observations with white noise, $\boldsymbol{\mathrm{v}}_k$ governed by a covariance matrix, ${}^L\boldsymbol{R}$. The pseudo velocity measurement is determined from estimation of the difference from current state (after update step) and the previous state, normalized by the time step. The velocity measurement is used since we found that data from the laser perception is accurate and helps to improve legs tracking accuracy. The prediction and update cycle of the KF for every track can be summarized by the following equation. For clarity, the indices are omitted in this equation.

\begin{IEEEeqnarray}{c}
$$
\boldsymbol{\tilde{\mathrm{x}}}=\boldsymbol{A}\boldsymbol{\hat{\mathrm{x}}}\IEEEyesnumber\IEEEyessubnumber*\\
\boldsymbol{\tilde{P}}=\boldsymbol{A\hat{P}A}^T + \boldsymbol{Q}\\
\boldsymbol{K}=\boldsymbol{\tilde{P}H}^T(\boldsymbol{H\tilde{P}H}+ \boldsymbol{R})^{-1}\\
\boldsymbol{\hat{\mathrm{x}}}=\boldsymbol{\tilde{\mathrm{x}}}+\boldsymbol{K}(\boldsymbol{\mathrm{z}}-\boldsymbol{H\tilde{\mathrm{x}}})\\
\boldsymbol{\hat{P}}=(\boldsymbol{I}-\boldsymbol{KH})\boldsymbol{\tilde{P}},
$$
\end{IEEEeqnarray}

where $\backsim$ and $\wedge$ denote the predicted and filtered quantities, $\boldsymbol{P}$ is the covariance matrix, $\boldsymbol{H}$ is the observation matrix relating the state to the observation, $\boldsymbol{I}$ is the identity matrix and $\boldsymbol{K}$ is the Kalman gain. The update steps require associations between predicted state and current observation, which is solvable by employing Munkres algorithm [@munkresalgo]. The cost matrix for the assignment problem is computed using the Mahalanobis distance, so that the filter uncertainties are taken into account. This method is also known as Global Nearest Neighbor. However, a global gating threshold setting is employed so that association pairs that are too far away will be dropped. The data association has three interpretations: leg(s) that are uniquely associated with a track, the legs that cannot be associated with any of the existing track(s), and the track(s) that cannot be associated to any identified leg. Each track has also confidence level which is increased exponentially (with a constant $\alpha$ as a parameter) if there is evidence. If there is no evidence, the confidence will decrease and the track is removed if the confidence is below certain threshold. The outcomes of the interpretations are summarized in Table. \ref{table:leg_tracking_outcome}. The ${}^Lc^j_k$ and ${}^Ld^j_k$ are the confidence of the leg track and the identified leg confidence associated with the leg track, with index $j$ at time $k$, respectively.

\begin{table}[!t]
\renewcommand{\arraystretch}{1.3}
\caption{Possible outcomes for leg tracking association}
\label{table:leg_tracking_outcome}
\centering
\footnotesize
\begin{tabular}{p{1.5cm}||p{6.cm}}
  \hline
    \textbf{Case} &
    \textbf{Outcome} \cr
  \hline\hline
    Assigned track with leg observation & Update the corresponding KF state. Track confidence is updated, ${}^Lc^j_k=\alpha \, {}^Lc^j_{k-1} + (1-\alpha) \, {}^Ld^j_{k}$\cr
  \hline
    Unassociated track & Skip update, but propagate the track using the KF predict step. Track confidence is degraded, ${}^Lc^j_k=\alpha \, {}^Lc^j_{k-1}$ \cr
  \hline
    Unassociated leg observation & Initiate a new track with zero velocity and zero confidence. \cr
  \hline
\end{tabular}
\end{table}



<!--
\begin{table}[!t]
\renewcommand{\arraystretch}{1.3}
\caption{Possible outcomes for leg tracking association}
\label{table:leg_tracking_outcome}
\centering
\footnotesize
\begin{tabular}{p{1.5cm}||p{6.cm}}
  \hline
    \textbf{Case} &
    \textbf{Outcome} \cr
  \hline\hline
    Uniquely assigned leg-track pair & Update the corresponding KF state. Track confidence is updated, ${}^Lc^j_k=\alpha \, {}^Lc^j_{k-1} + (1-\alpha) \, {}^Ld^j_{k}$\cr
  \hline
    Unassociated track(s) & Skip update, but propagate the track using the KF predict step. Track confidence is degraded, ${}^Lc^j_k=\alpha \, {}^Lc^j_{k-1}$ \cr
  \hline
    Unassociated leg(s) & Initiate a new track with zero velocity and confidence. \cr
  \hline
\end{tabular}
\end{table}
-->


### People Tracking

In order to track people, additional KFs are also created similarly with the formulation elaborated in the leg tracking method. The differences lie on the motion model, observation model and how the data association is treated. People tracking maintains a set of people tracks, ${}^P\boldsymbol{X}_k=\lbrace {}^P\boldsymbol{\mathrm{x}}^1_k, {}^P\boldsymbol{\mathrm{x}}^2_k,\dotsb,{}^P\boldsymbol{\mathrm{x}}^{{}^PN_k}_k \rbrace$, where ${}^PN_k$ is the number of people track at time $k$. Each leg track has a state estimate ${}^P\boldsymbol{\mathrm{x}}^j_k=[ x \: y \: \dot{x} \: \dot{y} \: \ddot{x} \: \ddot{y} ]^T$ of a target position, velocity, and acceleration in a 2D Cartesian coordinate. The rationale behind the constant acceleration model for the people tracking is that humans have walking pattern that accelerates and decelerates periodically. The observation model takes the states of the legs tracks as measurements, ${}^P\boldsymbol{\mathrm{z}}^m_k={}^L\boldsymbol{\mathrm{x}}^m_k$, where $m=1 \dotsb {}^LN_k$. With this another layer of KF tracker, any spurious leg identification can be robustly filtered out.

The people tracking observation model, ${}^P\boldsymbol{\mathrm{z}}_k={}^P\boldsymbol{H}{}^P\boldsymbol{\mathrm{x}}+\boldsymbol{\mathrm{v}}_k$ takes only position as measurement variable. If both legs of a person can be detected, the position is computed as the center of the two legs, otherwise it is coincide as the position of the only leg detected. The people associations is solved using the Munkres algorithm, but the observation targets are firstly categorized into three groups: the legs that has unambiguous nearest leg, the legs that has uncertain leg pair (because there are a few candidates nearby), and the legs that are located far enough to other legs. These groups are subsequently referred as the two-legs group, the ambiguous one-legs group, and the certain one-legs group, respectively. The grouping is determined based on simple search with a maximum distance between two-legs pair threshold parameter. The data association is solved using similar cost matrix method, however we found that for people tracking, real distance metric is more robust. This is possibly caused by the fact that the uncertainties of the KFs are not as accurate compared to the leg tracking, because the observation are virtual measurement (the state of the leg tracks). There are four interpretations of the people tracking association: two-legs group that are uniquely associated with a people track, two-legs group that are unassociated, the people track(s) that are paired with only one in the one-legs group, the people track(s) that are unpaired with any observation. The summary of the outcomes for these interpretations is shown in Table. \ref{table:people_tracking_outcome}. People tracks carry additional confidence information derived from its observation from leg tracking data. The confidence is update in a similar manner as in the leg tracking, with a constant parameter $\beta$. Two-legs group confidence is computed by averaging its legs confidences, and a single confidence is derived when a people track is associated with the one-legs group. If associated with the uncertain one-legs group, people track confidence are not updated, i.e. maintain the same confidence value as it is deemed ambiguous. The track will be removed if the confidence is below certain threshold. The ${}^Pc^j_k$ and ${}^Pd^j_k$ are the confidence of the people track and the confidence of the associated observation, with index $j$ at time $k$, respectively.

\begin{table}[!t]
\renewcommand{\arraystretch}{1.3}
\caption{Possible outcomes for people tracking association}
\label{table:people_tracking_outcome}
\centering
\footnotesize
\begin{tabular}{p{1.5cm}||p{6.cm}}
\hline
\textbf{Case} &
\textbf{Outcome} \cr
\hline\hline
Assigned people track with two-leg & Update the corresponding KF state. Track confidence is updated, ${}^Pc^j_k=\beta \, {}^Pc^j_{k-1} + (1-\beta) \, {}^Pd^j_{k}$\cr
\hline
Unassociated people track & Skip update, but propagate the track using the KF predict step. Track confidence is degraded, ${}^Pc^j_k=\beta \, {}^Pc^j_{k-1}$ \cr
\hline
Associated people track with one-leg & If the one-leg belongs to certain one-legs group, update the corresponding KF state and its confidence using the properties of the one-leg. \newline ,else if the one-leg is in the ambiguous group, the KF state is not updated, and the predict step is skipped for this track.\cr
\hline
Unassociated two-leg & Initiate a new track with zero velocity, zero acceleration and zero confidence. \cr
\hline
\end{tabular}
\end{table}


## Pursuit Controller

The pursuit controller drives ISERA towards the target person while avoiding obstacles. The driving control is based on the selection of discrete spatial zones with the lowest cost. An advantage of our approach is that no prior mapping is required in the pursue controller, hence, it could work on open outdoor as well as indoor areas. 

The pursuit controller makes use of the target given by the people tracking algorithm. Fig. \ref{fig_pursuit_controller} shows the variables used in the pursuit controller. The coordinate of the target person is described in polar coordinate with range $R_P$, and angle $\theta_{P}$ with respect to the robot heading. To determine the angle $\theta_{BP}$ where the robot should go to pursuit the target, the area surrounding the robot are first divided into $m$ number of spatial `buckets'' zones. The costs of each bucket are subject to evaluation, determined by factors such as the angle of each bucket to the target and to obstacles.

\begin{figure}[!t]
\centering
\includegraphics[width=3.5in]{PursuitIllustration.pdf}
\caption{Variables in the pursuit controller. (a) Illustration of the discrete spatial zone into $m$
number of buckets (9 buckets for illustration purpose). Note that
the nearest point to obstacle $C_{Bn}$ in bucket $n=\{1,...,m\}$ may
vary up depending on the environment (b) The parameters used in the
pursuit controller} \label{fig_pursuit_controller}
\end{figure}

A dynamic lookout distance is used to regulate the obstacle detection area where the lookout distance of each buckets $R_{OD}$, are kept to be less than the range to target. The area is shrunk and expanded proportional to the distance to the target $R_P$. In this way, the target person legs would not be regarded as an obstacle to be avoided.

The delta angle of each bucket with respect to the target
$\delta_{Bn}$ is calculated as,
\begin{equation}
\delta_{Bn} = |\theta_{Bn}-\theta_{P}|\,. \label{eq:delta}
\end{equation}
where $\theta_{Bn}$ is the angle of the center point in bucket $n$,
and $\theta_{P}$ is the target angle.

Then, the individual cost of each bucket is calculated as,
\begin{equation}
cost_{Bn}=\frac{\delta_{Bn}+1}{C_{Bn}}\,. \label{eq:cost}
\end{equation}
where $C_{Bn}$ is the distance to the nearest obstacle inside the
buckets.

Naturally, when there is no obstacle inside the obstacle detection range, the nearest bucket in the direction of the target would have the lowest cost. However, to prevent the robot from going too near into an obstacle, each bucket cost is also influenced by the cost of its neighboring several buckets. This averaging of costs could be seen as a repulsive effect to keep the robot away from the obstacles, while homing to the target pursuit. Finally, by comparing the average cost of each buckets, the center point of the bucket with the lowest average cost is chosen to be the pursuit heading $\theta_{BP}$.

The turn velocity $\omega_P$ is chosen to be proportional to pursuit heading angle $\theta_{BP}$,
\begin{equation}
\omega_P=K_{\omega}\times\theta_{BP}\,. \label{eq:cost}
\end{equation}
Where $K_{\omega}$ is a weighting constant.

The pursuit velocity $v_P$, is made to be proportional to obstacle clearance in the pursuit heading $C_{BP}$, the angle of the target person $\theta_P$, and the angle of the pursuit heading $\theta_{BP}$. For example, to pursuit with the highest speed, there should be no obstacle near the pursuit heading and the target person is in front of the robot. For other cases, the velocity will be reduced proportionally.

Another factor that has a significant impact on the robot behavior is the social zones of the person \cite{ref10}. As a service robot, the robot is expected to behave in the manner that is socially acceptable. The closer the robot to the target person, the more gentle the motion should be. The control laws combining these considerations are,

\begin{figure*}[!t]
\centering
\subfloat[Single target tracking]{\includegraphics[height=3.0in]{occluded-cropped.pdf}
\label{fig_static_result_one_people}}
\subfloat[Two targets tracking]{\includegraphics[height=3.0in]{occluded2.pdf}
\label{fig_static_result_two_people}}
\hfil
\subfloat{\includegraphics[width=1.2in]{legend.pdf}}
\caption{Tracking results for stationary robot.}
\label{fig_static_result}
\end{figure*}

\begin{equation}
v_P=\frac{C_{BP}}{K_o}\times\frac{\theta_m-\theta_P}{\theta_m}\times
\frac{\theta_m-\theta_{BP}}{\theta_m}\times D_P\,.
\label{eq:forwardvel}
\end{equation}
Where $K_o$ is a weighting constant, and $\theta_m$ is an angle between the front and the edge of the perception zone. $D_P$ is the parameter that changes the aggressiveness of the pursuit based on the social zone of the target person. In our ISERA, the typical values are,
\begin{eqnarray}
D_P =\hspace{-1mm} \left\{
\begin{array}{cc}
1 & \textrm{if } R_P>1.5m\\
0.6 & \textrm{if } 1m<R_P<1.5m\\
0.2 & \textrm{if } 0.3m<R_P<1m\\
0& \textrm{if } R_P<0.3m\\
\end{array}
\right.
\end{eqnarray}


# Experimentation and Results

The people detection, tracking, and pursuit algorithm has been tested both indoor and outdoor. The 2D Lidar has $180$ degree towards the front of the robot, which gives $720$ sampling points with $0.25$ degree separating each point. The observation of clusters for leg detection is limited to be within $5$ meters. Cluster size is limited to be within $0.07m - 0.2m$ and consist more than 4 sampling points. Clusters would be classified as legs if the confidence exceed a cut off threshold of $0.5$. For the pursuit controller, the area in front of the robot are divided into $72$ bucket zones. 

\begin{figure*}[!t]
\centering
\includegraphics[width=6.5in]{Following.pdf}
\caption{ISERA tracks and follows person A (wearing blue shirt), while person B (wearing black shirt) is passing in between: (a) Person B is about to pass through between person A and ISERA. (b) For a few moments ISERA did not have line of sight of person A, and the tracker will start to decay. (c) ISERA regain line of sight and continued following person A} \label{fig:blocked}
\vspace*{4pt}
\end{figure*}


## People Detection and Tracking

The first experiment is to verify the tracking ability, conducted while the robot is stationary. A single target tracking tests are coordinated with a person walking repeatedly from one start point and an end point following a line marked on the floor. The purpose of the line is to get the accuracy of the tracking result compared to the ground truth. Moreover, in a cluttered environment legs could be occluded by other people or obstacles. This condition is emulated by adding more people walking around while the primary target is still following the straight path. In order to verify multi-target tracking capability, tracking of two targets of interest were conducted. The paths are two straight lines located with a roughly $65cm$ distance to each other. 

The results of single and multiple target tracking are depicted in Fig.\ref{fig_static_result_one_people} and Fig.\ref{fig_static_result_two_people}, respectively. As can be seen, the tracking was executed robustly even in the middle of cluttered objects and some spurious leg measurements. It is observed that during occlusions, target can sometime deviate from the ground truth due to temporary uncertainties in the filter. Likewise, with two targets the walking patterns was opposite: one target person starts from one end of the path, the other start from the other end. Note that by design, the algorithm should initiate a new target (in people tracking), only when two legs are observed. After that, it will keep tracking the target even though only one leg is observed. By keeping the confidence level in each track, we can maintain which target to be preserved for the next filter cycle. If a target cannot be identified for certain period, the confidence will be drastically degraded overtime, and it will be removed.

<!--
\begin{figure}[!t]
\centering
\includegraphics[width=3.5in]{occluded.pdf}
\caption{Tracking result for one target.}
\label{fig_static_result_one_people}
\end{figure}

\begin{figure}[!t]
\centering
\includegraphics[width=3.5in]{occluded2.pdf}
\caption{Tracking result for two target.}
\label{fig_static_result_two_people}
\end{figure}
-->


## People Following

A series of experiments in a outdoor public area with obstacles to verify that ISERA can follow a person. The target person to be followed is selected by having the target person standing in front of ISERA. ISERA's pursuit controller maintains $30cm$ distance from the person. Some disturbance is emulated by having people walking around and in between ISERA and the target. 
The results shows ISERA consistently manage to track and follow the target person that walks below ISERA's maximum speed (limited to $1m/s$ in our hardware). When traveling between obstacles, ISERA also able to avoid going too close to the obstacle, and successfully follows the person. 

# Conclusion

In this paper, ISERA service robot algorithms to follow a target person have been presented. The person tracker associate the leg to a person and tracks based on the confidence level that is given by the leg identifier. The pursuit algorithm could regulate control between maintaining the distance to target person and to avoid obstacle. Future works should mitigate possible fail scenarios when the person walks too fast or being obstructed for a prolong period of time. 

<!--
[Figure 1](#bookmark=id.3acbajmbqepy)
[Figure 2](#bookmark=id.gu8nypnmx14m)
[Figure 3](#bookmark=id.qqvtpymv895n)
-->

