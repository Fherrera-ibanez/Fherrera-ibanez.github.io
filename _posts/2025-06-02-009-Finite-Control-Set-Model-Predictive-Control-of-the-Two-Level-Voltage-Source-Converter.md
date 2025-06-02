---
layout: distill
title: Finite Control Set Model Predictive Control of a Two Level Voltage Source Converter

authors:
  - name: Felipe Herrera

bibliography: BP0.bib

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

# Two-Level Voltage Source Converter

The 2L-VSC is the most basic inverter topology that every power electronics student learns. It consist of three legs with two switches each. The switches have two modes of operation: the *on* state and the *off* state. During the *on* state, the switch is conducting. Meanwhile, during the *off* state, the switch is blocking. To avoid the shoot-through state (which is when the two switches of one leg are in the *on* state, short-circuiting the DC-link bus), the switches operate in a complementary manner (i.e., when one switch is in the *on* state, the other is in the *off* state). The diagram of the 2L-VSC is shown in Fig. 1.  

<div class="row justify-content-sm-center">
  <div class="col-sm-8 mt-3 mt-md-0">
    {% include figure.liquid path="assets/img_bp0/2L-VSC.png" title="Fig. 1 Diagram of the 2L-VSC" caption="Fig. 1 Diagram of the 2L-VSC" class="img-fluid rounded z-depth-1" %}
  </div>
</div>

The mathematical model of the 2L-VSC will be posed using the formulation presented in <d-cite key="mora_computationally_2019"></d-cite>. In this post, the switch states are represented by variables $u_x$ with $x \in \{a,b,c\}$. Considering that we have two states *per leg*, and three legs, the number of possible combinations available for the 2L-VSC (given that the switches must operate in a complementary manner) are 8. This fact is important for the operation of the FCS-MPC strategy. The complete operation of the converter can be characterized by the three-phase switching vector $\boldsymbol u_{abc} = [u_a\;u_b\;u_c]^\intercal \in \mathbb U  {\triangleq} \{1,0\}^3$. The set $\mathbb U$ contain the eight combinations of the switching vector, and is defined as:

$$\mathbb U \triangleq \left\{\begin{bmatrix}0\\0\\0\end{bmatrix},\begin{bmatrix}1\\0\\0\end{bmatrix},\begin{bmatrix}0\\1\\0\end{bmatrix},\begin{bmatrix}1\\1\\0\end{bmatrix},\begin{bmatrix}0\\0\\1\end{bmatrix},\begin{bmatrix}1\\0\\1\end{bmatrix},\begin{bmatrix}0\\1\\1\end{bmatrix},\begin{bmatrix}1\\1\\1\end{bmatrix}\right\}$$

Until now, we have just characterized the operation of the converter. However, we are interested in the converter output voltage. Consider that an ideal *dc* voltage source is connected at the *dc* side of the converter. The *ac* side output voltage is then:

$$\boldsymbol v_{abc} = V_{dc}\boldsymbol u_{abc}$$
