---
layout: distill
title: Current Control of a Two Level Voltage Source Converter with Finite Control Set MPC

authors:
  - name: Felipe Herrera

bibliography: 2025-02-06-BP0.bib

# Optionally, you can add a table of contents to your post.
# NOTES:
#   - make sure that TOC names match the actual section names
#     for hyperlinks within the post to work correctly.
#   - we may want to automate TOC generation in the future using
#     jekyll-toc plugin (https://github.com/toshimaru/jekyll-toc).
toc:
  - name: Learning Outcomes
    # if a section has subsections, you can add them as follows:
    # subsections:
    #   - name: Example Child Subsection 1
    #   - name: Example Child Subsection 2
  - name: Introduction
  - name: Two-Level Voltage Source Converter
  - name: Finite Control Set MPC
  - name: Simulation Results


---

**Disclaimer: In my work, I use MATLAB/Simulink in conjunction with the PLECS Blockset for power electronics simulation and control algorithm implementation, as these tools are widely used in academic settings. I understand that these are licensed tools and may not be accessible to everyone. Unfortunately, due to time constraints, I am currently unable to port the control algorithms to C/C++ or to open-source platforms. I may consider doing so in the future when time permits, but please understand that this will not always be possible.**

# Learning Outcomes
- Understand the principles of Finite Control Set Model Predictive Control (FCS-MPC).
- Learn to apply these principles to control the Two-Level Voltage Source Converter (2L-VSC).

# Introduction

**Model Predictive Control (MPC)** has become a widely researched topic in power electronics. In this field, two main approaches dominate: **Continuous Control Set MPC (CCS-MPC)** and **Finite Control Set MPC (FCS-MPC)**.

The key difference lies in how they generate switching signals:

- **CCS-MPC** uses a **modulator** to translate continuous control actions into switching signals.
- **FCS-MPC**, on the other hand, **directly selects switching states** during the optimization process.

Here’s a simplified overview of how FCS-MPC works:

1. A **mathematical model** of the load connected to the converter is developed using differential equations. This model depends on the converter’s switching states.
2. The model is **discretized** so it can run on a digital controller. It predicts how the system will behave under different switching actions.
3. A **cost function** is defined to numerically evaluate how well each switching action performs according to control objectives.

In early implementations of FCS-MPC, the controller **evaluated all possible switching states** using the cost function and selected the one that **minimized** it. That optimal switching state was then applied to the converter.

Don’t worry if this sounds abstract — this is just a high-level summary. In the sections that follow, we’ll break down the core ideas and walk through the basic implementation of an FCS-MPC algorithm.

This post is based on the paper:

*J. Rodriguez _et al_., "Predictive Current Control of a Voltage Source Inverter," in _IEEE Transactions on Industrial Electronics_, vol. 54, no. 1, pp. 495-503, Feb. 2007, doi: 10.1109/TIE.2006.888802.*

# Two-Level Voltage Source Converter

The 2L-VSC is the most basic inverter topology that every power electronics student learns. It consist of three legs with two switches each. The switches have two modes of operation: the *on* state and the *off* state. During the *on* state, the switch is conducting. Meanwhile, during the *off* state, the switch is blocking. To avoid the shoot-through state (which is when the two switches of one leg are in the *on* state, short-circuiting the DC-link bus), the switches operate in a complementary manner (i.e., when one switch is in the *on* state, the other is in the *off* state). The diagram of the 2L-VSC is shown in Fig. 1.  

<div class="row justify-content-sm-center">
  <div class="col-sm-8 mt-3 mt-md-0">
    {% include figure.liquid path="assets/img/img_bp0/2L-VSC.svg" title="Fig. 1 Diagram of the 2L-VSC" caption="Fig. 1 Diagram of the 2L-VSC" class="img-fluid rounded z-depth-1" %}
  </div>
</div>

The mathematical model of the 2L-VSC will be posed using the formulation presented in <d-cite key="mora_computationally_2019"></d-cite>. In this post, the switch states are represented by variables $u_x$ with $x \in [a,b,c]$. Considering that we have two states *per leg*, and three legs, the number of possible combinations available for the 2L-VSC (given that the switches must operate in a complementary manner) are 8. This fact is important for the operation of the FCS-MPC strategy. The complete operation of the converter can be characterized by the three-phase switching vector $\boldsymbol u_{abc} = [u_a\;u_b\;u_c]^\intercal \in \mathbb U  {\triangleq} [1,0]^3$. The set $\mathbb U$ contain the eight combinations of the switching vector, and is defined as:


$$\mathbb U \triangleq \left\{\begin{bmatrix}0\\0\\0\end{bmatrix},\begin{bmatrix}1\\0\\0\end{bmatrix},\begin{bmatrix}0\\1\\0\end{bmatrix},\begin{bmatrix}1\\1\\0\end{bmatrix},\begin{bmatrix}0\\0\\1\end{bmatrix},\begin{bmatrix}1\\0\\1\end{bmatrix},\begin{bmatrix}0\\1\\1\end{bmatrix},\begin{bmatrix}1\\1\\1\end{bmatrix}\right\}$$


Until now, we have just characterized the operation of the converter. However, we are interested in the converter output voltage. Consider that an ideal *dc* voltage source is connected at the *dc* side of the converter. The *ac* side output voltage is then:

$$\boldsymbol v_{abc} = V_{dc}\boldsymbol u_{abc}$$

It is typical to transform this three-phase voltage into a two-phase voltage using the *Amplitude-invariant Clarke transform*. To do so we define the transformation as follows:

$$
\pmb{\mathcal{T}} = \frac{2}{3}\begin{bmatrix} 1 & -\frac{1}{2} & -\frac{1}{2}\\ 0 & \frac{\sqrt{3}}{2} & \frac{-\sqrt{3}}{2} \end{bmatrix}
$$

Then, the transformed voltage vector is $\boldsymbol v_s = \pmb{\mathcal{T}}\boldsymbol v_{abc}$. Into the eight combinations of the new transformed voltage vectors we have two zero vectors (i.e., the voltage produced is zero), and six active vectors (i.e., the voltage produced is different than zero). The different voltage vectors can be visualized in the space vector diagram shown in Fig. 2. 

<div class="row justify-content-sm-center">
  <div class="col-sm-8 mt-3 mt-md-0">
    {% include figure.liquid path="assets/img/img_bp0/SV_diagram_2LVSC.svg" title="Fig. 2 Space Vector diagram of the 2L-VSC" caption="Fig. 2 Space Vector Diagram of the 2L-VSC" class="img-fluid rounded z-depth-1" %}
  </div>
</div>

This is it for the modelling of the converter, lets go now to the control algorithm.

# Finite Control Set MPC

The idea behind FCS-MPC is simple. Predict the evolution of the load variables subjected to each switching vector of the converter, and choose the one which produce the best result based on some criteria. Until now we have not discussed the load, and we will keep it that way for now. It will be introduced briefly.  

Consider the following optimization problem:

$$
\begin{aligned}
	\underset{\boldsymbol u_s \in \pmb{\mathcal{T}}\mathbb U}{\operatorname*{arg\,min}} \quad & J(\boldsymbol u_s)
\end{aligned}
$$

We will be considering this simple problem for now. If you are not familiarized with this notation, it can seem a little bit daunting. Lets break it down:
1. $\boldsymbol u_s$ is the optimization variable. It is the input to the problem, and its also what we can manipulate ($\boldsymbol u_s$ is the transformed three-phase switching vector $\boldsymbol u_s = \pmb{\mathcal{T}}\boldsymbol u_{abc}$).
2. We have a new set $\mathbb V \triangleq \pmb{\mathcal{T}}\mathbb U$. This set contains the elements of set $\mathbb U$ multiplied by the transformation matrix $\pmb{\mathcal{T}}$. 
3. $J(\boldsymbol u_s)$ is the cost function. It defines the measure of how wrong or far off a solution is from what we want. 
4. Third we have ${\operatorname*{arg\,min}}$. This means that the output of the problem is the element of $\mathbb V$, $\boldsymbol u_s^\star$, which minimizes the cost function $J$.

Here we aren't considering constraints, they will be studied in future posts. To consider this example, we will now consider the load and it will be in this case a simple resistive-inductive (*R-L*) load. It consist of the series connection of a resistance and an inductor. We will not go into the details of the load modelling, as it will be assumed known knowledge. The dynamics of the load currents are defined by the following differential equation:

$$
\frac{\boldsymbol di_s}{dt} = -\frac{R}{L}\boldsymbol i_s + \underbrace{\frac{V_{dc}}{L}\boldsymbol u_s}_{\frac{1}{L}\boldsymbol v_s} 
$$

Where $\boldsymbol i_s = [i_{s\alpha}\;i_{s\beta}]^\intercal \in \mathbb R^2$ is the load current, *R* is the load resistance, *L* is the load inductance, and $V_{dc}$ is the *dc*-link voltage. The algorithm requires a discretization of this differential equation. For this there are several methods, however, we will just consider *Forward Euler*:

$$
\frac{d\boldsymbol x(t)}{dt} \approx \frac{\boldsymbol x_{k+1}-\boldsymbol x_k}{T_s}
$$

Where $T_s$ is the step size or sampling interval. Applying this into the dynamics of the load current we obtain:

$$
\boldsymbol i_{s,k+1} = \left(1-\frac{R}{L}T_s\right)\boldsymbol i_{s,k} + \frac{V_{dc}}{L}T_s\boldsymbol u_{sj,k} 
$$

Notice that now we have $\boldsymbol u_{sj,k}$. ¿What does *j* stands for there?. Well, ¿remember that the set $\mathbb U$ has eight elements (and by extension, so does set $\mathbb V$)?. Variable *j* is just the index of the elements of the set. Thus, $j\in \{0,\ldots,7\}$ (indexing from 0). 
¿What should be the objective of our algorithm? There could be several, depending on the specific application. To keep it simple, we will just make the load current follow a given time-variable reference. To comply with this objective, our cost function is the following:

$$
J = \lVert \boldsymbol i_{s,k+1} - \boldsymbol i_{s,k+1}^{ref}\rVert_2^2
$$

Again, lets break this down. $\lVert \star \rVert_2^2$ is just the $\ell_2$-norm. To define it, consider the *n*-dimensional vector $\boldsymbol \zeta = \begin{bmatrix} \zeta_1 & \ldots & \zeta_n \end{bmatrix}$. The $\ell_2$-norm is defined as $\lVert \boldsymbol \zeta \rVert_2^2 = \zeta_1^2 + \ldots + \zeta_n^2 = \boldsymbol \zeta^\intercal \boldsymbol \zeta$. For the specific case we are seeing, this would translate to:

$$
J = \left(i_{s\alpha,k+1} - i_{s\alpha,k+1}^{ref} \right)^2 + \left(i_{s\beta,k+1} - i_{s\beta,k+1}^{ref} \right)^2 
$$

There is a discussion about the choice of norm in the literature, we will talk a bit about in the future. The discussion is already settled by the results obtain to this date, so it will be just an informative post. Lets rewrite the cost function in terms of $\boldsymbol u_s$ replacing the dynamics of $\boldsymbol i_{s,k+1}$ into the cost function. We obtain:

$$
J(\boldsymbol u_{sj}) = \lVert \boldsymbol u_{sj} - \boldsymbol u_{db}\rVert_2^2
$$

Where $\boldsymbol u_{db}$ is the control action required to reach $\boldsymbol i_{s,k+1}$ from $\boldsymbol i_{s,k}$ in one time-step. It is defined as follows:

$$
\boldsymbol u_{db} = \frac{L}{V_{dc}T_s}\left[\boldsymbol i_{s,k+1}^{ref} - \left(1-\frac{R}{L}T_s \right)\boldsymbol i_{s,k}\right]
$$

To obtain it, just make $\boldsymbol i_{s,k+1}=\boldsymbol i_{s,k+1}^{ref}$ in the prediction equation of the load current, and solve for $\boldsymbol u_{sj,k}$. Finally, a simple flowchart implementation of the FCS-MPC discussed can be seen in Fig. 3.

<div class="row justify-content-sm-center">
  <div class="col-sm-8 mt-3 mt-md-0">
    {% include figure.liquid path="assets/img/img_bp0/FCSMPC_FC.svg" title="Fig. 3 Flow Chart of FCS-MPC Algorithm" caption="Fig. 3 Flow Chart of the FCS-MPC algorithm" class="img-fluid rounded z-depth-1" %}
  </div>
</div>

This flowchart is adapted from the one presented in <d-cite key="rodriguez_state_2013"></d-cite>. 

# Simulation results

The parameters for the load are $R = 0.5\; \Omega$, $L = 10\;mH$, $V_{dc} = 100\;V$ and the sampling frequency 10 kHz. The results with a sinusoidal reference signal are shown in Fig. 4.

<div class="row justify-content-sm-center">
  <div class="col-sm-8 mt-3 mt-md-0">
    {% include figure.liquid path="assets/img/img_bp0/Sinusoidal.png" title="Fig. 4 Simulation results with sinusoidal reference" caption="Fig. 4 Simulation results with sinusoidal reference" class="img-fluid rounded z-depth-1" %}
  </div>
</div>

Meanwhile the results with a triangular reference signal are shown in Fig. 5.

<div class="row justify-content-sm-center">
  <div class="col-sm-8 mt-3 mt-md-0">
    {% include figure.liquid path="assets/img/img_bp0/Triangular.png" title="Fig. 5 Simulation results with triangular reference" caption="Fig. 5 Simulation results with triangular reference" class="img-fluid rounded z-depth-1" %}
  </div>
</div>
