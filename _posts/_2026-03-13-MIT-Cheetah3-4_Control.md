---
title: "MIT Cheetah 3 Locomotion in Simulation <i>– Control Log</i>"
tagline: "A step-by-step implementation in MuJoCo (Go2)"
excerpt: "MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped Robot"
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
MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped Robot
</p>

## 목표
Go2 모델이 균형을 잡으면서 걸을 수 있게 하는 것

## 구조
구조는 크게 목표값을 산출하는 Command Generation항과 Controller항으로 나눌 수 있다. 

**1. Controller** : 보행제어기는 크게 몸의 균형을 잡기 위한 네발의 지면반력을 계산하는 ***Force Control***과 다리를 움직이기 위한 ***Swing control***로 구성된다. 
먼저 제어 모델은 다음과 같이 로봇의 질량중심(COM) 평행 이동 가속도 $\ddot{\boldsymbol{p}}_c$와 본체 각가속도 $\dot{\boldsymbol{\omega}}_b$, 그리고 로봇의 네 발 각각에 작용하는 힘 $\boldsymbol{F}=\left(\boldsymbol{F}_1^T, \boldsymbol{F}_2^T, \boldsymbol{F}_3^T, \boldsymbol{F}_4^T\right)^T$ 사이의 일반적으로 사용되는 선형 관계식을 사용한다:

<div class="sm"></div>

$$
\underbrace{\left[\begin{array}{ccc}
\mathbf{I}_3 & \ldots & \mathbf{I}_3 \\
{\left[\boldsymbol{p}_1-\boldsymbol{p}_c\right] \times} & \ldots & {\left[\boldsymbol{p}_4-\boldsymbol{p}_c\right] \times}
\end{array}\right]}_{\boldsymbol{A}} \boldsymbol{F}=\underbrace{\left[\begin{array}{c}
m\left(\ddot{\boldsymbol{p}}_c+\boldsymbol{g}\right) \\
\boldsymbol{I}_G \dot{\boldsymbol{\omega}}_b
\end{array}\right]}_{\boldsymbol{b}}
$$

<div class="sm"></div>

논문의 Cheetah3는 로봇 질량의 6%밖에 안되는 네 다리(총 2.7kg)를 장착시켜 다리의 영향을 무시하고 제어 모델을 단순화할수 있도록 했다. 그래서 땅을 지지하고 있는 발에서 오는 지면반력을 계획하기 용이했다. 그러나 Go2의 경우 다리가 총 약 8.28kg으로 로봇 전체 질량의 54.5%나 되어 다리를 무시하면 목표 지면반력을 잘못 계산하는 문제가 생긴다. 그래서 무게 중심이 관여되는 변수들( $\mathbf{p}_{c}$, $\dot{p}_c$, $I_G$ ) 모두 몸통 중심이 아닌 전신에 대한 값으로 바꿔 계산한다.  

**2. Command ** :
1) Virtual Predictive Support Polygon
- a virtual support polygon is defined to provide a desired CoM location that generalizes across all gaits.

2) 목표발 위치 계산
- Raibert Heuristic + Capture Point 방식으로 계산
- Bézier curve 로 발 궤적 계산 

## 방식
### Virtual Predictive Support Polygon
- 로봇이 중심을 잡기 위해 발 위치값을 반영해 목표 중심값을 계산한다.
- 계산 알고리즘 설명
  


### Force Control (Balance controller)
Force control은 로봇의 질량 중심, 몸통의 회전에 대해 PD제어기로 구성되어 있다. 발높이값은 상수값으로.............. (nominal height of locomotion)

>> at 70% extension, a typical configuration during operation

이 높이는 몸통과 발위치값 사이 고정값이므로 특정 지형에서는 몸높이를 더 피거나 웅크릴 수 있도록 제어하는 부분이 들어가야할것 같다. 여기서는 단순 평지에서의 보행이므로 다리를 다폈을때의 70%의 높이를 상수값 z0(nominal height of locomotion)로 두기로 한다. balance controller에서는 목표 pcom(pc,d)의 z축요소를 posture adjustment 에서 추정된 지면높이(a₀ + a₁x + a₂y)로부터 몸통 중심지점(p_com)에서 z0만큼 올라간 값으로 두어 적절한 다리 굽힘상태로 중심을 잡도록 제어한다. 

$$
\left[\begin{array}{c}
\ddot{\boldsymbol{p}}_{c, d} \\
\dot{\boldsymbol{\omega}}_{b, d}
\end{array}\right]=\left[\begin{array}{c}
\boldsymbol{K}_{p, p}\left(\boldsymbol{p}_{c, d}-\boldsymbol{p}_c\right)+\boldsymbol{K}_{d, p}\left(\dot{\boldsymbol{p}}_{c, d}-\dot{\boldsymbol{p}}_c\right) \\
\boldsymbol{K}_{p, \omega} \log \left(\boldsymbol{R}_d \boldsymbol{R}^T\right)+\boldsymbol{K}_{d, \omega}\left(\boldsymbol{\omega}_{b, d}-\boldsymbol{\omega}\right)
\end{array}\right]
$$

목표 각가속도의 경우 회전행렬간의 오차를 지수맵(exponential map)으로 구해 P제어기에 넣음.

제어기는 관절에 대한 Impedance control이 포함되어 있다.




The goal of the Balance Controller is to resolve an optimal distribution of leg forces F that drive the approximate COM dynamics to the corresponding desired dynamics given by

$$
\boldsymbol{b}_d=\left[\begin{array}{c}
m\left(\ddot{\boldsymbol{p}}_{c, d}+\boldsymbol{g}\right) \\
\boldsymbol{I}_G \dot{\boldsymbol{\omega}}_{b, d}
\end{array}\right]
$$



사용자 입력인 pdot_d 와 yaw rate은 desired 회전행렬과 omega 만들때 반영된다.
pdot_d, yaw rate 모두 바디좌표계 기준 명령이고 수식(2)의 병진운동 명령(translational cmd) 생성과 회전운동 명령 생성에 각각 이용된다. pdot_d는 병진운동 명령에 사용하기 위해 몸통의 pitch,roll 회전에 대한 자유도는 구속하고, 전진방향에 반영될 yaw_d로만 회전행렬을 만들어 월드 좌표계로 변환한다.






### 1. 수식 변환 (OSQP 표준형 맞추기)

OSQP의 기본 최적화 수식은 다음과 같습니다.

$$\min_x \frac{1}{2}x^T P x + q^T x$$

$$\text{s.t.} \quad l \le A_{osqp} x \le u$$

문서의 수식 (4)는 다음과 같습니다.

$$F^{*} = \min_{F \in \mathbb{R}^{12}} (AF - b_d)^T S (AF - b_d) + \alpha ||F||^2 + \beta ||F - F_{prev}^*||^2$$

$$\text{s.t.} \quad CF \le d$$

여기서 우리가 구하려는 변수 $x$는 12차원의 힘 벡터 $F$입니다. 위 목적 함수를 전개하여 $F$에 대한 2차항($P$)과 1차항($q$)으로 묶어내야 합니다.

- **행렬 $P$ (2차항 계수 도출):**
    
    목적 함수를 전개하면 $F^T A^T S A F + \alpha F^T F + \beta F^T F$ 형태의 2차항이 나옵니다.
    
    OSQP는 앞에 $\frac{1}{2}$이 곱해져 있으므로, 계수에 2를 곱해줘야 합니다.
    
    $$P = 2(A^T S A + (\alpha + \beta)I)$$
    
    (여기서 $I$는 $12 \times 12$ 단위 행렬입니다.)
    
- **벡터 $q$ (1차항 계수 도출):**
    
    $F$가 한 번만 곱해진 1차항들을 전개하여 모으면 다음과 같습니다.
    $$q = -2(A^T S b_d + \beta F_{prev}^*)$$
    
- **제약 조건 행렬 및 경계 ($A_{osqp}, l, u$):** 
	- 문서의 제약 조건은 마찰 뿔(Friction pyramid)과 수직 항력의 한계를 나타내는 $CF \le d$ 입니다. 이를 OSQP 형태인 $l \le A_{osqp} x \le u$ 에 맞추면 다음과 같습니다.
	    - $A_{osqp} = C$
	    - $u = d$
	    - $l = [-\infty, -\infty, \dots, -\infty]^T$ (부등식이 $\le$ 형태만 있으므로 하한은 무한대입니다.)



- Balance controll 검증

<details>
  <summary><b>Balance controll 검증 외력교란 <i>(펼치기)</i></b></summary>
  <div markdown="1">
네, MuJoCo에서 외력을 가하는 방법이 여러 가지 있습니다.

**`mj_applyFtAndTrq` 또는 `xfrc_applied`** 를 사용하면 특정 body에 원하는 시점에 원하는 크기의 force/torque를 직접 인가할 수 있어요. 예를 들어:

```python
# body에 x방향으로 100N 힘을 0.1초간 가하기
if 1.0 < sim_time < 1.1:
    data.xfrc_applied[body_id] = [100, 0, 0, 0, 0, 0]  # [fx, fy, fz, tx, ty, tz]
else:
    data.xfrc_applied[body_id] = [0, 0, 0, 0, 0, 0]
```

이게 가장 간단하고 직관적인 방법이에요. 방향, 크기, 지속시간을 자유롭게 조절 가능하고, impulse(짧고 강한 힘)나 sustained push(지속적인 힘) 모두 구현할 수 있습니다.

그래서 테스트 시나리오를 이렇게 구성할 수 있어요:

- 정상 standing 상태에서 측면 push (lateral robustness)
- 정상 standing 상태에서 전방/후방 push
- 다양한 크기의 힘으로 push해서 balance controller가 버틸 수 있는 한계 force를 측정
- 반복적인 랜덤 방향 push

Balance controller의 성능 지표로 복원 시간, 최대 허용 교란 크기, body orientation 오차 등을 측정하면 꽤 의미 있는 검증이 됩니다.

바로 구현 시작할까요?

  </div>
</details>

<div class="sl"></div>

## Reference
***M. Focchi et al., “High-slope terrain locomotion for torque-controlled quadruped robots,” (2017)*** <br>
***B. Stephens et al., “Push recovery by stepping for humanoid robots with force controlled joints,” (2010)***




<!-- 1. Go2 로봇의 질량 분석 (XML 모델 기반)
XML 파일에 정의된 각 링크의 관성(inertial) 데이터를 모두 더한 결과입니다.

몸통(Base) 질량: 6.921 kg

다리 1개당 질량: 약 2.071 kg

Hip (abduction): 0.678 kg

Thigh (hip): 1.152 kg

Calf (knee): 0.241 kg

다리 4개 총 질량: 약 8.285 kg (2.071 kg × 4)

로봇 총 질량: 15.206 kg (6.921 kg + 8.285 kg)

👉 결론: Go2의 다리 질량 비중은 **54.5%**이며, 오히려 몸통 질량(45.5%)보다 무겁습니다.

2. 논문 속 로봇들과의 비교
이 논문들이 다리 무게를 무시할 수 있었던 이유는 로봇의 하드웨어 특성이 Go2와 완전히 달랐기 때문입니다.


Cheetah 3: 다리 링크를 알루미늄으로 가공하여 4개의 다리 총 질량이 2.7kg에 불과합니다. 이는 전체 질량(45kg)의 단 6% 수준이므로 다리를 무시하는 제어 모델 단순화가 합리적이었습니다.


HyQ (High-slope 논문): 로봇 전체 질량 75kg 중 몸통 질량이 47kg을 차지하여 몸통 비중이 훨씬 큽니다. 다리가 움직일 때 CoM의 오차를 측정해본 결과 1cm 내외에 불과했기 때문에 몸통 위치를 CoM으로 근사할 수 있었습니다.
+1

3. Go2에 모델 단순화를 그대로 적용할 경우의 문제점
다리가 로봇 무게의 절반 이상을 차지하는 Go2에서 목표 상태 $\mathbf{p}_{c}$를 단순히 '몸통의 중심'으로만 가정하고 제어기를 구현하면 치명적인 문제가 발생합니다.

다리 하나를 공중으로 들어 앞으로 뻗는 순간, 무거운 다리 무게 때문에 로봇의 **실제 전신 무게중심(True CoM)**이 몸통 중심에서 다리가 뻗은 방향으로 크게 이동합니다.

하지만 제어기(QP 솔버)는 여전히 무게중심이 몸통 한가운데 있다고 착각하고 목표 지지 반력(GRF)을 계산합니다. 그 결과, 잘못된 모멘트 팔(Lever arm) 길이를 바탕으로 토크를 분배하게 되어 로봇이 균형을 잃고 쓰러지게 됩니다. -->