---
title: "MIT Cheetah 3 Locomotion in Simulation <i>– 2.Gait Scheduler</i>"
tagline: "A step-by-step implementation in MuJoCo (Go2)"
excerpt: "Designing a Hybrid Scheduler for Integrated Periodic Phase-Based and Event-Based Control"
categories:
  - Locomotion
tags:
  - locomotion
  - control
author_profile: false
header:
  image: /assets/images/header/chatg3_1200x600.png
  teaser: /assets/images/header/gaitschedule.png
---

<p style="text-align: left; color: #7d8590; font-size: 1.0rem; margin-top: -1.1.1rem; margin-bottom: 2rem; font-style: italic;">
Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains
</p>

## 목표
동물의 주기적 걸음걸이 패턴과 예상치 못한 접촉에 대한 Gait Scheduler를 만드는것. 네발동물의 걸음걸이 형태는 범주화되어있으며 그 중 "Trot"을 구현한다. 

<img src="/assets/images/posts/mit-cheetah3/p2.png" alt="Gait Scheduler" style="width: 100%; display: block; margin: 0 auto;">




<div class="ss"></div>

아래에 다양한 걸음걸이에 대해 정리했다:

<div style="max-width: 600px; margin: 0 auto;">
  <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden;">
    <iframe
      src="https://www.youtube.com/embed/WrR3fVQ3W3s?si=XwLlPjY2s0o-foy3&amp;"
      style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; border: 0;"
      allowfullscreen>
    </iframe>
  </div>
</div>
<br>

<details>
  <summary><b>사족보행 동물의 주요 보행 패턴 <i>(펼치기)</i></b></summary>
  <div markdown="1">

### 1. 트로트 (Trot, 속보)
- 패턴: 대각선 방향의 두 다리가 한 쌍(Pair)이 되어 동시에 가동(예: 왼쪽 앞다리와 오른쪽 뒷다리의 동시 착지 및 이륙).
- 특징: 지지점의 중간 지점이 로봇의 질량 중심(CoM) 근처에 위치하여 안정성이 뛰어남. 전후, 좌우, 대각선 이동 및 제자리 회전 등 방향 전환이 자유로움.
- 한계 및 변형: 고속 주행 시 다리 간 충돌 위험으로 속도 제한이 있음. 이를 극복하기 위해 모든 다리가 지면에서 떨어지는 비행 구간(Flight period)을 추가한 **플라잉 트로트(Flying-trot)**로 최고 속도 향상이 가능.

### 2. 페이스 (Pace, 측대보)
- 패턴: 같은 방향 측면의 두 다리가 한 쌍이 되어 움직임(예: 왼쪽 앞·뒷다리의 동시 가동).
- 특징: 다리가 긴 동물이 저속 보행 시 앞·뒷다리의 간섭을 방지하기 위해 주로 사용. 균형 유지를 위해 몸통 폭을 좁히거나 착지 시 다리를 안쪽으로 기울여 질량 중심 아래를 지지해야 함.

### 3. 바운드 (Bound, 비약보)
- 패턴: 두 앞다리와 두 뒷다리가 각각 쌍을 이루어 차례로 착지 및 이륙하는 패턴.
- 특징: 트로트보다 보행 주기(Gait frequency)가 빠르고 비행 시간이 길어 고속 주행에 유리.
- 한계: 질량 중심 바로 아래를 지지하지 못해 몸통이 앞뒤로 크게 흔들리는 피칭(Pitching) 현상 발생. 뒷다리의 가동 범위를 확보하기 위해 앞다리의 보폭을 좁게 유지해야 함.

### 4. 갤럽 (Gallop, 습보)
- 패턴: 다리들이 쌍을 이루지 않고 순차적으로 교차하며 지면을 딛는 방식. 로터리 갤럽(Rotary gallop)의 경우 '우측 뒷다리 → 양 뒷다리 → 좌측 뒷다리 → 좌측 앞뒷다리 → 좌측 앞다리 → 양 앞다리 → 우측 앞다리' 순으로 복잡하게 전환됨.
- 특징: 초고속 주행에 가장 적합한 패턴. MIT Cheetah 3 로봇의 경우 갤럽 방식을 통해 바운드보다 부드러운 주행과 최고 속도 3m/s를 달성.

### 5. 프론크 (Pronk, 껑충뛰기)
- 패턴: 네 다리가 동시에 착지하고 동시에 이륙하는 짧은 도약을 반복.
- 특징: 명확한 지지 구간(예: 150ms)과 긴 비행 구간(예: 350ms)이 구분됨. 제자리 점프를 통한 자세 안정화가 가능.

### 6. 1족 보행 패턴 (One-Foot Gaits)
- 패턴: 한 번에 오직 하나의 다리만 지면에 닿으며, 각 지지 구간 사이에 반드시 비행 구간이 존재(예: 좌전 → 우후 → 우전 → 좌후 순서).
- 특징: 이론상 가능하나, 다리 간 충돌 방지 및 질량 중심 부근 정밀 착지가 기구학적으로 매우 어려움. 매우 유연한 척추 구조가 필요하여 실제 로봇이나 생물체에서는 드물게 관찰됨.

사족보행 로봇은 속도, 지형, 에너지 효율, 하드웨어 구조(다리 길이, 관절 범위)에 최적화된 보행 패턴을 전략적으로 선택하여 주행함.

  </div>
</details>

<div class="sl"></div>

## 방식
### 1. Periodic Phase-based Gait

먼저 기본적인 동물의 걸음걸이 패턴을 선택하여 주기적인 gait scheduler로 만든다.

<img src="/assets/images/posts/mit-cheetah3/gait_planner.png" alt="Gait Scheduler" style="width: 60%; display: block; margin: 0 auto;">

논문에 제시된 이 phase map의 경우, 오른앞다리와 왼뒷다리, 왼앞다리와 오른뒷다리가 한쌍으로 같은 주기를 가지고 스윙과 컨택을 한다. 대각선 방향의 두다리가 한쌍이 되어 걷는 방식인 Trot으로 볼수 있다. 

<div class="sl"></div>

### 2. Event-Based Gait Switching

장애물이나 높낮이 변화가 있는 거친 지형(Unstructured terrain)에서 유연하게 대응하며 걸을 수 있도록 이벤트 기반 보행전환기를 추가한다. 주기적 보행루틴을 따른다면 로봇의 발이 아직 공중에 있는데 스케줄상 땅을 디뎌야 할 시간이 되었을때 발을 허공에서 강하게 내리찍게 된다. 반대로 이미 예상보다 일찍 땅에 닿았는데도 스케줄상 여전히 스윙(Swing) 단계라면, 제어기가 발을 계속 뻗으려고 하면서 로봇이 크게 비틀거리거나 불안정해질수 있다. 그래서 고정된 시간에 얽매이지 않고, 로봇이 신뢰할 수 있는 접촉 감지(Contact detection)를 통해 예상보다 지면에 일찍 닿거나(Early contact) 늦게 닿는(Late contact) 상황에 유연하게 제어기를 바꾸도록 함. 예를 들어, 스윙 중에 예상치 못하게 땅에 닿는 순간 즉시 궤적 추종을 멈추고 현재 발 위치를 유지함으로써, 발이 튕겨 오르는 현상(Bounce)을 막고 안정적으로 자세를 잡게 해 준다.

<div class="smm"></div>

> The phased scheduled state, $s_\phi$, remains the underlying mechanism for choosing the gait pattern. However, leg control uses the estimated contact state, $$\hat{s}$$, coming from the algorithm. A simple Event-Based Finite State Machine in Figure 10 shows the states and transition conditions.

<div class="sl"></div>

```
Early   : s_hat = 1, s_phi = 0
Swing   : s_hat = 0, s_phi = 0
Contact : s_hat = 1, s_phi = 1 
Late    : s_hat = 0, s_phi = 1

Swing -> Early   : s_hat = 1
Swing -> Late    :  t > t_bar{c} + t_0
Swing -> Contact : t = t_bar{c} + t_0 , s_hat = 1
Early -> Contact :  t >= t_bar{c} + t_0
Late -> Contact  : s_hat = 1
Contact -> Swing : t >= t_c + t_0
```
<div class="sm"></div>

상태를 Early, Swing, Contact, Late 네가지로 만들고 전환에 대한 조건들을 적용한다. 이렇게 주기적인 보행패턴에서 나온 $s_\phi$와 앞서 ***1.Leg detection*** 포스팅에서 계산된 상태추정값 $$\hat{s}$$을 가지고 Finite State Machine을 구성한다.

<div class="sl"></div>

## Reference

***M. H. Raibert, "Legged robots that balance," (1986)***