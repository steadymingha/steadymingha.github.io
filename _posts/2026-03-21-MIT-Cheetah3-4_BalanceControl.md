---
title: "MIT Cheetah 3 Locomotion in Simulation <i>– 3.Force Control</i>"
tagline: "A step-by-step implementation in MuJoCo (Go2)"
excerpt: "High-slope terrain locomotion for torque-controlled quadruped robots"
categories:
  - Locomotion
tags:
  - locomotion
  - control
author_profile: false
header:
  image: /assets/images/header/chatg2_1200x600.png
  teaser: /assets/images/header/chatg2_1600x600.png
---


<p style="text-align: left; color: #7d8590; font-size: 0.9rem; margin-top: -1.1.1rem; margin-bottom: 2rem; font-style: italic;">
High-slope Terrain Locomotion for Torque-Controlled Quadruped Robots
</p>

## 목표

Go2 모델이 균형을 잡기 위해 네 발에 적절히 무게를 실을 수 있게 하는 것. 움직이지 않고 지지하는 발로만 균형을 잡는다.

## 구조

**1. State Estimation**
<br>
제어기 구현 및 성능 확인을 우선적으로 진행하고자 몸통 위치 및 속도값은 mujoco에서 제공하는 값을 이용했다.

**2. Centroidal Dynamics Model**
<br>
Cheetah3 는 다리 총 질량이 전체 질량의 6%밖에 안되어 이를 무시하고 Centroidal Dynamics Model을 구성했으나 Go2는 다리가 매우 무거우므로 무시할 수 없어 몸통의 중심이 아닌 전체 로봇의 중심값을 CoM으로 두었다.

$$
\underbrace{\left[\begin{array}{ccc}
\mathbf{I}_3 & \ldots & \mathbf{I}_3 \\
{\left[\boldsymbol{p}_1-\boldsymbol{p}_c\right] \times} & \ldots & {\left[\boldsymbol{p}_4-\boldsymbol{p}_c\right] \times}
\end{array}\right]}_{\boldsymbol{A}} \boldsymbol{F}=\underbrace{\left[\begin{array}{c}
m\left(\ddot{\boldsymbol{p}}_c+\boldsymbol{g}\right) \\
\boldsymbol{I}_G \dot{\boldsymbol{\omega}}_b
\end{array}\right]}_{\boldsymbol{b}}
$$

<br>
**3. Desired CoM Location**
<br>
네 발의 위치값으로 만든 다각형(Virtual Predictive Support Polygon)의 중심을 몸통 중심의 목표위치로 둔다. 
<br>

**4. QP Force Distribution**
<br>
로봇의 질량중심(CoM)위치와 몸의 회전자세에 대해 PD제어기로 목표가속도 값을 만들고 이를 질량과 관성모멘트로 힘을 구성한다. QP에서는 이 힘을 최적화할 지면반력을 계산한다. 사용하는 힘을 최소화하고, 힘의 변화도 최소화 하는 목표도 함께 반영한다.

$$
\left[\begin{array}{c}
\ddot{\boldsymbol{p}}_{c, d} \\
\dot{\boldsymbol{\omega}}_{b, d}
\end{array}\right]=\left[\begin{array}{c}
\boldsymbol{K}_{p, p}\left(\boldsymbol{p}_{c, d}-\boldsymbol{p}_c\right)+\boldsymbol{K}_{d, p}\left(\dot{\boldsymbol{p}}_{c, d}-\dot{\boldsymbol{p}}_c\right) \\
\boldsymbol{K}_{p, \omega} \log \left(\boldsymbol{R}_d \boldsymbol{R}^T\right)+\boldsymbol{K}_{d, \omega}\left(\boldsymbol{\omega}_{b, d}-\boldsymbol{\omega}\right)
\end{array}\right]
$$

<br>

$$
\begin{aligned}
\boldsymbol{F}^*=\min _{\boldsymbol{F} \in \mathbb{R}^{12}} & \left(\boldsymbol{A} \boldsymbol{F}-\boldsymbol{b}_d\right)^T \boldsymbol{S}\left(\boldsymbol{A} \boldsymbol{F}-\boldsymbol{b}_d\right) +\alpha\|\boldsymbol{F}\|^2+\beta\left\|\boldsymbol{F}-\boldsymbol{F}_{\mathrm{prev}}^*\right\|^2 \\
\text { s.t. } & \boldsymbol{C F} \leq \boldsymbol{d}
\end{aligned}
$$

<br>

**4. Torque Mapping**
<br>
구해진 지면반력을 토크로 전환한다. 지면반력은 땅이 발을 미는 방향이므로 로봇이 미는 방향으로 바꾸기 위해 부호를 -로 붙인다.

$$
\tau_{f f}=-S J_c^{\top} f^d
$$

## 구현

먼저 서있는 상태에서 제어를 해야하므로 처음 1.5초까지는 관절 PD제어기를 작동시켰고 이후는 관절 PD제어기를 끄고 QP에서 계산된 토크값만 들어가도록 했다. 3초부터 왼쪽 옆구리를 밀고, 오른쪽 옆구리를 밀고, 엉덩이쪽을 미는 가상의 외력을 넣어서 밸런스 제어기를 검증하였다. 관절에 대한 제어기가 들어가있지 않은 상태여서 외력에 버티면서 다리가 점점 모이는 것을 볼 수 있다. 


<div class="sm"></div>


<div style="max-width: 600px; margin: 0 auto;">
  <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden;">
    <iframe
      src="https://www.youtube.com/watch?v=pajeQIi7p_M"
      style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; border: 0;"
      allowfullscreen>
    </iframe>
  </div>
</div>

<div class="sm"></div>

몸통에 대한 병진제어 게인값을 올리면 몸통이 회전없이 버틴다.


<div style="max-width: 600px; margin: 0 auto;">
  <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden;">
    <iframe
      src="https://www.youtube.com/watch?v=RPde4mT83FI"
      style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; border: 0;"
      allowfullscreen>
    </iframe>
  </div>
</div>

<br>

__High-slope terrain locomotion for torque-controlled quadruped robots__ 논문 에서는 QP 토크값에 PD Joint controller도 추가해서 밸런스 제어입력으로 넣는데 Cheetah3에서는 블록다이어그램에만 적혀있고 따로 언급하지 않아 넣지 않았다. 그러나 지지발이 밀리는것으로 봐서 기본적으로 들어가는것이 맞다고 보여진다. 