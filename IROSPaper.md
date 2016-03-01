# Introduction



Recently, with increasing human-robot interaction applications, service robots have received more and more attentions. Mobile service robots designed to assist people in their daily lives, such as robotic butlers, robotic waiters and robotic pets, which once are the stuffs of since fictions, are becoming reality. In these robots, the capability to autonomously detect, track and follow a person has been regarded as an important functionality of the robotic systems. However, until now, it still remains a challenge that endowing this capability to these robots when they move in a cluttered environment.

In previous studies, the approaches of autonomous person following can be classified as two categories: vision based and laser based. Due to the low cost, RGB-D cameras are widely adopted in the vision based approaches as the work presented in \cite{ref0}\cite{ref1}. However, the minimum distance requirement and the narrow field view of the RGB-D sensors limit the applications for following persons.

The laser based approaches have an advantage in terms of large field of view and robustness on any lighting condition. \cite{ref3} may be the first to propose an approach for autonomous tracking people by using laser data. In this work, owing to the priori information of the operation environment, human is easy to be
tracked by a mobile robot. In \cite{ref4}, leg detection in laser scans makes it possible for mobile robots detection and tracking both stationary and moving people. Based on \cite{ref4}, in the approach presented in \cite{ref5}, motion of the target people is modeled to enhance the tracking ability of an autonomous robot. To avoid the tediously manual tuning for determining the sizes of leg clusters in the laser scan, data-driven methods are employed in \cite{ref6} \cite{LuS13} to leaning shapes of leg clusters automatically. Nevertheless, enhancing the robust of the learning approaches for detecting and tacking moving people is still under research. More recently, \cite{ref7} introduces a framework for outdoor person tracking and following for a smart wheel chair robot by extending a trained legs detector from \cite{ref8}. In \cite{ref9}\cite{ref10}, robotic behaviors in relation to the speed and distance to the person being followed have been analyzed. However, the correlation from detecting legs into tracking a person, and the robustness when multiple person is around the person being followed has not been explored in these studies. In additional, few of researchers fuse the visual information and laser data for tracking people \cite{KobilarovSHB06}\cite{M.Kleinehagenbrock}\cite{ref2}, but this kind of scheme may complicate the tracking system.


\begin{figure}[!t]
\centering
\includegraphics[width=3.5in]{ISERA.pdf}
\caption{(a) ISERA robot in action of tracking and following a
target person (b) The sensor and interface of ISERA service
robot}\label{fig_isera}
\end{figure}


In this paper, we propose a novel approach to autonomously detect, track and follow persons for a service robot, namely ISERA shown in Fig. \ref{fig_isera}, in a cluttered environment. In this approach, a tuneable 2D leg detection method is developed to regulate the strictness of the leg identifier as needed. The proposed approach then tracks the leg clusters, and associate the data to form and track a virtual target
person. Two layers of Kalman filter is used to ensure the robustness of the person tracking despite close proximity to another person and temporary line of sight loss to the leg clusters occurred. Another notable merit of the proposed approach is that a real time obstacle avoidance algorithm is embedded into the feed-back motion controller of the robot. Hence, experiments show ISERA can follow a target person while avoid obstacles autonomously in a cluttered environment.

The paper is organized as follows. Section II describes the mobile robot system. The people detection, tracking and following approach is detailed in Section III. The experiment and results described in Section VI shows the valid and effective of the proposed approach. Finally, a conclusion is given in Section V.




# System Description

The ISERA robot is designed as a human friendly service robot that
could serve as a butler or courier in public area. The robot could
communicate with human through its auditory microphone arrays and
touch screen on board. ISERA is built on the top of a differential
drive mobile platform with a maximum speed of $1m/s$. A master PC is
installed inside the robot for various algorithms implementation
such as 2D mapping, autonomous navigation, human tracking and human
robot interaction.

Although ISERA is also equipped with an RGB-D camera, the Hokuyo
UTM-30-LX lidar is the only sensor that is being used for person
identification, tracking, and following proposed in this paper. The
lidar is mounted about $30cm$ from the ground, which corresponds to
sensor reading of legs below the knee, around the shank area of
average adults.


# Person Following


## Leg Identification

In this paper, we proposes a leg identification that build based on
the principle of circle geometric fitting [Premebida 2005], but
instead of giving a crisp output, our algorithm calculates the
confidence of a segmented cluster to be considered as a leg. The
confidence will then be used in the tracking algorithm to determine
the decay rate of the tracker. Another addition is the algorithm
could also identify a joint dual leg, where the two legs of a person
standing could be very close to each other and seen as one large
cluster by the sensor.

### Clustering
As a start, data from the lidar processed to find clusters with
certain maximum and minimum size and positions. In practice when
using our lidar, where there are distortions and noise when looking
into an object from certain direction, a segment is considered a
cluster only if there at least two consecutive jumps from the
adjacent points that exceeded a threshold at both the start and end
of the segment.

To reduce false clusters at the background of another cluster, we
also added an obstruction filtering constraint, where a cluster
could only spawn when there are no other objects that are both
nearer and exactly next to the points where the cluster starts and
ends.

In our experiment, cluster position is limited to be within $5$
meters. Cluster size is limited to be less than the size of two legs
next to each other (typically $0.4m$), larger than the size of a
single leg (typically $0.07m$) and consist more than 4 points.


### Leg Segmentation

#### Dual leg filtering
Dual leg filtering is done to clusters that exceeded the maximum
size for a single leg (typically 0.18m). First, the local minimum
points of the cluster with respect to the cluster edges are
searched. To search the minimum points, the distance between each
point in the cluster with a virtual line that starts and ends at the
cluster edges is calculated. The local minimum point of the cluster
is the point where it has shorter distance to the virtual line
compared to several of its neighboring left and right points.

A cluster is concluded to be a dual leg if there is a local minimum
that crosses the virtual line and the size of the cluster is larger
than a maximum size of one leg. Otherwise, the cluster will be
discarded from the leg segmentation process.

As for the clusters that are concluded to be a dual leg clusters,
they are dissected at the minimum point into two parts. The part
with more sampling points is now considered as a new single leg
cluster candidate and will be processed further. The other part with
less sampling point is discarded from the leg segmentation process.


#### Modeling and Evaluation of clusters
To be an arc or convex shape, the middle point of the cluster must
be within a reasonable distance (typically between 0.1-0.7) from the
virtual line of the cluster edges. The single leg cluster that
passes this filter will be processed further, otherwise it will be
discarded. This filtering quickly discards any non-convex shape
cluster with minimum computation cost.

For clusters that has passed the previous criterion and filters, the
center point of the cluster is approximated as the virtual point in
the middle of the cluster leftmost and rightmost edges. For each
cluster, the distance between the lidar and the center point of that
cluster is denoted as $z$ and will be used as a variable in the
analysis.

To analyze the single leg clusters, three criterions are used as
benchmark. First, the mean error of the inscribe angle variant (IAV)
of the cluster []. The second is the error of the standard deviation
of the cluster. The third is related to the error in the size of the
cluster.

The benchmark criterion is taken from numerous measurement of dummy
humanoid with typical cylindrical legs at varying distance $z$ to
the lidar. The data collected are then interpolated into a function
of distance $z$ and kept as a benchmark values.

Next, the data collected the actual measurements single leg clusters
are compared with their benchmark. Then, based on the normalized
errors between the desired benchmark and the actual measurement, an
equation to formulate the confidence of a cluster to be considered
as a leg is derived as,
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
Where $Leg_{c}$, which has a value between $0$ and $1$, is the
confidence of cluster $c$ to be considered a leg. $I_c$, $S_c$, and
$P_c$ are IAV mean, IAV standard deviation, and number of points
that are calculated from sensor data, respectively. $I_d(z)$,
$S_d(z)$,and $P_d(z)$ are the desired values of the IAV mean, IAV
standard deviation, and number of points in the cluster for distance
$z$, respectively. $K_I$, $K_S$, and $K_P$ are weighting factor that
are tuneable based on our preferred strictness of the leg
segmentation. Strict leg segmentation gives the least number of
false positives, and could be used when the lidar identifies mostly
bare legs. On the other hand, if the circumstance allows us to
tolerate more false positives in the leg detection, loose
segmentation could be used to detect people that are wearing
baggy/loose pants.

Finally, for clusters that has a confidence that exceed a cut off
threshold (typically $Leg_{c}>0.5$), the center point and the
confidence value are passed on to the leg and person tracking
algorithm described in the next subsection.




## Leg and People Tracking

In order to follow certain person, multiple target tracking is developed to estimate positions of both detected legs and people. The estimation is implemented using Kalman Filters (KFs) [@KalmanRudolfEmil] with two different motion model for legs and for people. In general, the state estimation of the legs and people can be modeled by a linear system, $\boldsymbol{\mathrm{x}}_{k+1}=\boldsymbol{A} \boldsymbol{\mathrm{x}}_k + \boldsymbol{\mathrm{w}}_k$. The process noise, $\boldsymbol{\mathrm{w}}_k$ is modelled as Gaussian white noise with covariance matrix $\boldsymbol{Q}$. The difference between the legs and people estimation is the matrix $\boldsymbol{A}$. The left superscripts $L$ and $P$, denote the entities for the linear systems correspond to legs and people state models (e.g. ${}^L\boldsymbol{A}$ and ${}^P\boldsymbol{A}$), respectively.  The input for the legs tracking are the leg detection results in each frame $k$. Furthermore, the input for the people tracking is the leg tracks states, treated as observations. This way the tracked people states can always be maintained, instead of creating temporary tracks in each update step [@ref7]. In every received data frame, all KFs tracks are updated. The result of the people tracking is transmitted to the pursuit controller (Sec. \ref{pursuit-controller}), so that the robot can follow certain person. The process can be summarized in Fig. \ref{fig_process_diagram}.

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{process_diagram.pdf}
\caption{Tracking process.}
\label{fig_process_diagram}
\end{figure}



### Legs Tracking
The KF for the legs uses a constant velocity motion model with a pseudo velocity measurement during the KF update steps. At discrete time $k$, the KF maintains a set of leg tracks, ${}^L\boldsymbol{X}_k=\lbrace {}^L\boldsymbol{\mathrm{x}}^1_k, {}^L\boldsymbol{\mathrm{x}}^2_k,\dotsb,{}^L\boldsymbol{\mathrm{x}}^{{}^LN_k}_k \rbrace$, where ${}^LN_k$ is the number of leg track at time $k$. Each leg track has a state estimate ${}^L\boldsymbol{\mathrm{x}}^j_k=[ x \: y \: \dot{x} \: \dot{y} ]^T$ of a target position and velocity in a 2D Cartesian coordinate. During the update step of the Kalman Filter, leg identification are processed using an observation model, ${}^L\boldsymbol{\mathrm{z}}_k={}^L\boldsymbol{H}{}^L\boldsymbol{\mathrm{x}}_k + \boldsymbol{\mathrm{v}}_k$. The observation includes position and velocity observations with white noise, $\boldsymbol{\mathrm{v}}_k$ governed by a covariance matrix, ${}^L\boldsymbol{R}$. The pseudo velocity measurement is determined from estimation of the difference from current state (after update step) and the previous state, normalized by the time step. The velocity measurement is used since we found that data from the laser perception is accurate enough and helps to improve legs track accuracy. The predict and update cycle of the KF for every track can be summarized by the following equation. Note that the indices are not displayed in this equation to avoid clutter.

\begin{IEEEeqnarray}{c}
$$
\boldsymbol{\tilde{\mathrm{x}}}=\boldsymbol{A}\boldsymbol{\hat{\mathrm{x}}}\IEEEyesnumber\IEEEyessubnumber*\\
\boldsymbol{\tilde{P}}=\boldsymbol{A\hat{P}A}^T + \boldsymbol{Q}\\
\boldsymbol{K}=\boldsymbol{\tilde{P}H}^T(\boldsymbol{H\tilde{P}H}+ \boldsymbol{R})^{-1}\\
\boldsymbol{\hat{\mathrm{x}}}=\boldsymbol{\tilde{\mathrm{x}}}+\boldsymbol{K}(\boldsymbol{\mathrm{z}}-\boldsymbol{H\tilde{\mathrm{x}}})\\
\boldsymbol{\hat{P}}=(\boldsymbol{I}-\boldsymbol{KH})\boldsymbol{\tilde{P}},
$$
\end{IEEEeqnarray}

where $\backsim$ and $\wedge$ denote the predicted and filtered quantities, $\boldsymbol{P}$ is the covariance matrix, $\boldsymbol{H}$ is the observation matrix relating the state to the observation, $\boldsymbol{I}$ is the identity matrix and $\boldsymbol{K}$ is the Kalman gain. The update steps requires associations between predicted state and current observation, which is solvable by employing Munkres algorithm [@munkresalgo]. The cost matrix for the assignment problem is computed using the Mahalanobis distance, so that the filter uncertainties is taken into account. This method is also known as Global Nearest Neighbor. However, a global gating threshold setting is employed so that association pairs that are too far away will be dropped. The data association has three interpretations: leg(s) that are uniquely associated with a track, the legs that cannot be associated with any of the existing track(s), and the track(s) that cannot be associated to any identified leg. Each track has also confidence level which is increased exponentially (with a constant $\alpha$ as a parameter) if there is evidence, and decreased, if there is not. The track is removed if the confidence is below certain threshold. The outcome of the interpretations are summarized in Table. \ref{table:leg_tracking_outcome}. The ${}^Lc^j_k$ and ${}^Ld^j_k$ are the confidence of the leg track and the identified leg confidence associated with the leg track, with index $j$ at time $k$, respectively.

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

### People Tracking

In order to track people, additional KFs are also created similarly with the formulation elaborated in the leg tracking method with the superscript indices $P$ and it is not included to save space. The differences lie on the motion model, observation model and how the data association is treated. People tracking maintains a set of people tracks, ${}^P\boldsymbol{X}_k=\lbrace {}^P\boldsymbol{\mathrm{x}}^1_k, {}^P\boldsymbol{\mathrm{x}}^2_k,\dotsb,{}^P\boldsymbol{\mathrm{x}}^{{}^PN_k}_k \rbrace$, where ${}^PN_k$ is the number of people track at time $k$. Each leg track has a state estimate ${}^P\boldsymbol{\mathrm{x}}^j_k=[ x \: y \: \dot{x} \: \dot{y} \: \ddot{x} \: \ddot{y} ]^T$ of a target position, velocity, and acceleration in a 2D Cartesian coordinate. The rationale behind the constant acceleration model for the people tracking is that humans have walking pattern that accelerates and decelerates periodically. The observation model takes the states of the legs tracks as measurements, ${}^P\boldsymbol{\mathrm{z}}^m_k={}^L\boldsymbol{\mathrm{x}}^m_k$, where $m=1 \dotsb {}^LN_k$. With this another layer of KF tracker, any spurious leg identification can be robustly filtered out.

The people tracking observation model, ${}^P\boldsymbol{\mathrm{z}}_k={}^P\boldsymbol{H}{}^P\boldsymbol{\mathrm{x}}+\boldsymbol{\mathrm{v}}_k$ takes only position as measurement variable. Since the two legs of a person can be detected, the position is computed as the center of the two legs, otherwise it is the same as the position of the only leg detected. The people associations is solved using the Munkres algorithm, but the observation targets are firstly categorized into three groups: the legs that has unambiguous nearest leg, the legs that has uncertain leg pair (because there are a few candidates nearby), and the legs that are located far enough to other legs. These groups are subsequently referred as the two-legs group, the ambiguous one-legs group, and the certain one-legs group, respectively. The grouping are determined based on simple search with a maximum distance between two-legs pair threshold parameter. The data association is solved using similar cost matrix method, however we found that for people tracking, real distance metric is more robust. This is possibly caused by the fact that the uncertainties of the KFs are not as accurate compared to the leg tracking, because the observation are virtual measurement (the state of the leg tracks). There are four interpretations of the people tracking association: two-legs group that are uniquely associated with a people track, two-legs group that are unassociated, the people track(s) that are paired with only one in the one-legs group, the people track(s) that are unpaired with any observation. The summary of the outcomes for these interpretations is shown in Table. \ref{table:people_tracking_outcome}. People tracks carry additional confidence information derived from its observation from leg tracking data. The confidence is update in a similar manner as in the leg tracking, with a constant parameter $\beta$. Two-legs group confidence is computed by averaging its legs confidences, and a single confidence is derived when a people track is associated with the one-legs group. If associated with the uncertain one-legs group, people track confidence are not updated, i.e. maintain the same confidence value as it is deemed ambiguous. From our experiments, it is better to leave the confidence constant until a new observation improve the state. The track is removed if the confidence is below certain threshold. The ${}^Pc^j_k$ and ${}^Pd^j_k$ are the confidence of the people track and the confidence of the associated observation, with index $j$ at time $k$, respectively.


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

Our controller drives ISERA while avoiding obstacles based on the
selection of discrete spatial zones with the lowest cost. An
advantage of our approach is that no prior mapping is required in
the pursue controller, the controller simply process its surrounding
online. Hence, it could work on large outdoor areas as well as
indoor.

The pursuit controller makes use of the target given by the tracker.
First the coordinate of the target person is transformed into a
polar coordinate with range $R_P$, and angle $\theta_{Bn}$ with
respect to the robot heading. To determine the angle $\theta_{BP}$
where the robot should go to pursuit the target, the zones
surrounding the robot are first divided into $m$ number of
``buckets'', and the costs of each bucket are subject to evaluation.
The costs of the buckets are determined by several factors, such as
how far is each bucket to the target, and how clear is the bucket
from obstacles.


\begin{figure}[!t]
\centering
\includegraphics[width=3.5in]{PursuitIllustration.pdf}
\caption{(a) Illustration of the discrete spatial zone into $m$
number of buckets (9 buckets for illustration purpose). Note that
the nearest point to obstacle $C_{Bn}$ in bucket $n=\{1,...,m\}$ may
vary up depending on the environment (b) The parameters used in the
pursuit controller} \label{fig_stanley}
\end{figure}


An obstacle detection with a dynamic area size is used to maintain
homing behavior to the target person it chases while avoiding any
obstacles on his path,. In order to limit the obstacle detection
area, the lookout distance of each bucket is limited by a constant.
However, whenever the distance to the target $R_P$ is smaller
compared to the constant, the observation range $R_{OD}$ will shrink
to be slightly less compared to the distance to the target. And
whenever the target moves away from the robot, the observation range
will be expanded up to the limit. In this way, the people being
followed will not be regarded as an obstacle.

Due to design constrains, the perception zone of the lidar on board
is limited to $180$ degree towards the front of the robot, which
gives $720$ sampling points with $0.25$ degree separating each
point. In our experiment, we discretely divide the observation into
$72$ buckets. With each bucket starts right next to each other, we
have divided the laser perception zone into mini $2.5$ degree
buckets. Next, for each bucket, the nearest distance to obstacle
inside the observation range $C_{Bn}$ is stored to be the depth of
that bucket.

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

Naturally, when there is no obstacle inside the obstacle detection
range, the nearest bucket in the direction of the target would have
the lowest cost. However, to prevent the robot from going too near
into an obstacle, each bucket cost is also influenced by the cost of
its neighboring several buckets. This averaging of costs could be
seen as a repulsive effect to keep the robot away from the
obstacles, while homing to the target pursuit. Finally, by comparing
the average cost of each buckets, the center point of the bucket
with the lowest average cost is chosen to be the pursuit heading
$\theta_{BP}$.

As an additional safety measure for operating in human environment
where people can suddenly come towards the robot, a short range
virtual bumper from the laser reading is used to halt the robot when
any obstacles come within its boundary. Once the obstacle or
pedestrian have moved away, the robot will continue the target
pursuit, provided that the target has not disappeared from the
robot's point of view.

The pursuit forward velocity $v_P$ and turn velocity $\omega_P$ are
calculated based on several factors. The turn velocity is simply
proportional to pursuit heading angle $\theta_{BP}$,
\begin{equation}
\omega_P=K_{\omega}\times\theta_{BP}\,. \label{eq:cost}
\end{equation}
Where $K_{\omega}$ is a weighting constant.

For the pursuit velocity, the factors are how far the obstacle
clearance in the pursuit heading $C_{BP}$, the angle of the target
person $\theta_P$, and the angle of the pursuit heading
$\theta_{BP}$. For example, to chase with the highest speed, there
should be no obstacle near the pursuit heading and the target person
is in front of the robot. For other cases, the velocity will be
reduced proportionally.

Another factor that has a significant impact on the robot behavior
is the social zones of the person []. As a service robot, the robot
is expected to behave in the manner that is socially acceptable. The
closer the robot to the target person, the more gentle the motion
should be. The control laws combining these considerations are,

\begin{equation}
v_P=\frac{C_{BP}}{K_o}\times\frac{\theta_m-\theta_P}{\theta_m}\times
\frac{\theta_m-\theta_{BP}}{\theta_m}\times D_P\,.
\label{eq:forwardvel}
\end{equation}
Where $K_o$ is a weighting constant, and $\theta_m$ is an angle
between the front and the edge of the perception zone. $D_P$ is the
parameter that changes the aggressiveness of the pursuit based on
the social zone of the target person. In our ISERA, the typical
values are,
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

## People Detection and Tracking Experiment

### Procedure

### Results


## People Following Experiment

### Procedure
In order to validate that the proposed method could tracks and
follows a person in a dynamic environment, we conducted a series of
experiments in a large outdoor public area with obstacles. No
mapping has been done for this environment prior to the experiment.

First, the target person is introduced to ISERA by standing in front
of the robot, showing both legs, and tapping a button on the
interface to initiate the tracking. Starting from this handshake
procedure, ISERA now tries to follow up to $30cm$ from the person.
Besides normal procedure of steadily following the person, we also
deliberately introduce some disturbance in the experiment by having
people walking around and in between ISERA and its target person.


### Results
We verified in our experiment results that ISERA consistently manage
to track and follow the target person that walks below ISERA's
maximum speed (limited to $1m/s$ in our hardware). As long as line
of sight to target person is maintained, ISERA has no issue
following the person. When traveling between static obstacles, ISERA
also avoided going to close to the obstacle, and successfully
follows the person.

There are two failure scenario related to the loss of line of sight
that occurred during experiment. First scenario is when a person
suddenly moved and deliberately blocked ISERA's path. Without
sufficient time to react, the range became too close and the safety
feature kicked in to completely stopped ISERA until the obstacle
moved away. In theory, despite the disturbance and blockage by other
persons, ISERA is able to regain the tracking as long as the target
person tracker has not fully decayed (see Fig. \ref{fig:blocked}).
Typically, the target person tracker could remain for a few seconds,
depending of the confidence level when the line of sight is lost.
The second scenario, ISERA lost track when the target person is
turning sharply ($>90$ degree turn) over an obstacle, and if ISERA
was lagging behind the obstacle, the line of sight was lost for too
long due to the blockage and the target tracker faded away.


\begin{figure}[!t]
\centering
\includegraphics[width=3.5in]{Following.pdf}
\caption{ISERA tracks and follows person A (wearing blue shirt),
while person B (wearing black shirt) is passing in between:(a)
Person B is about to pass through between person A and ISERA (b) For
a few moments ISERA did not have line of sight of person A, and the
tracker will start to decay. (c) ISERA regain line of sight and
continued following person A} \label{fig:blocked}
\end{figure}



In the proposed method described in this paper, ISERA has no other
information to correlate that a particular leg belongs to a
particular person aside from the 2D lidar data and the tracker
algorithm. To improve and mitigate the dependency of maintaining
line of sight, in our future work we intend to combine vision as an
additional means in recognizing the target person specifications,
and to experiment with $360$ degree lidar for more information on
the person segmentation.


# Conclusion

In this paper, we have presented our ISERA service robot algorithms
to follow a target person. Our leg identifier offers a flexibility
of strictness in the criterion for different circumstances. The
person tracker associate the leg to a person and tracks based on the
confidence level that is given by the leg identifier. Lastly, our
pursuit algorithm is independent of map, regulate the gentleness of
the approach, and could be used to avoid obstacle while pursuing the
target person.

The work presented in this paper is a step towards a service robot
that intelligently and politely follows a person in crowded public
spaces. Such robot would have limitless potential application in the
area of human service robotics.




<!--
[Figure 1](#bookmark=id.3acbajmbqepy)
[Figure 2](#bookmark=id.gu8nypnmx14m)
[Figure 3](#bookmark=id.qqvtpymv895n)
-->

