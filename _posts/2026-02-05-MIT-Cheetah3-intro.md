---
title: "MIT Cheetah 3 Locomotion Control in Simulation"
tagline: "A step-by-step implementation in MuJoCo (Go2)"
excerpt: "Implemented in MuJoCo with the Unitree Go2 model"
categories:
  - fun
tags:
  - locomotion
  - control
author_profile: false
header:
  image: /assets/images/header1.jpg
  teaser: /assets/images/header_1.jpg
---



## 목적

사족보행 로봇에 관심이 생기면서 가장 base가 될만한 논문을 찾았는데 그 중 하나가 "MIT Cheetah 3- Design and Control of a Robust, Dynamic Quadruped Robot" 였다. 사족보행로봇의 메커니즘을 이해하기 위해 이 논문을 구현해보기로 결정. 

## MIT Cheetah3 소개

이 논문은 기존의 유압방식과 직렬 탄성 구동기를 사용하여 제어하는 방식과는 달리 낮은 기어비의 기어박스와 고토크 모터를 결합하여 전류와 조인트 위치만으로 지면반력을 추정하는 방식을 채택했음. 
기존 방식은 무겁거나 반응속도(Bandwidth)와 최대 토크를 희생하는 방식이었으나 MIT Cheetah 3는 과감하게 '힘의 정확도(Force Accuracy)'보다는 '높은 반응성(High Bandwidth)'을 선택함. 이를 통해 복잡한 센서 없이도 지면 반력을 즉각적으로 느끼고 반응할 수있는 설계 철학으로서의 'Proprioceptive Actuation' 방식을 제시했다.

### High Bandwidth를 어떻게 구현했나?:
**1. 하드웨어: 낮은 기어비와 스프링 제거 (Low Gear Ratio & No Springs)**
  - 낮은 기어비 (Low Gear Ratio): 일반적인 로봇 팔은 100:1 이상의 높은 기어비를 써서 작은 모터로 큰 힘을 내지만, 마찰이 심해 외부 충격을 모터가 느끼지 못함. 반면 Cheetah 3는 7.67:1이라는 매우 낮은 기어비의 단일 스테이지 유성 기어(Single-stage planetary gear)를 사용.

  - 스프링 제거 (No Series Elasticity): SEA와 달리 모터와 다리 사이에 스프링이 없음

  - 결과 (역구동성, Backdrivability): 이 설계 덕분에 로봇 발이 땅에 닿는 순간, 그 충격(힘)이 기어 마찰이나 스프링에 흡수되지 않고 곧바로 모터의 회전축으로 전달되어 물리적 반응속도를 비약적으로 높임



**2. 제어 아키텍처: 초고속 전류 제어 루프 (High-Speed Control Loop)**

  - 20 kHz (전류 제어): 가장 밑단의 모터 드라이버는 무려 20,000Hz (0.05ms) 속도로 전류(토크)를 제어함. 이는 모터가 느끼는 미세한 힘의 변화에 즉각적으로 반응하여 원하는 힘을 낼 수 있게 하기 위함.

  - 4.5 kHz (다리 제어): 임피던스 제어(Impedance Control)나 관절 PD 제어 루프주기는 4.5kHz. 이는 일반적인 로봇 제어 주기(1kHz)보다 훨씬 빠르며, 매우 딱딱한(Stiff) 접촉 상황에서도 안정성을 유지할 수 있다.

  - 1 kHz (보행 제어): 전체적인 보행 계획(Locomotion) 루프 주기

**3. 기구학: 다리의 경량화 (Low Inertia Legs)**

  - 반응성을 좋게 하기 위해 Cheetah 3는 무거운 모터를 전부 몸통 쪽(Hip)에 몰아넣고, 무릎 관절은 가벼운 체인이나 링크로 구동함.

  - 이로 인해 다리 링크(Link)들의 무게는 2.7kg으로, 로봇 전체 무게의 **6%**밖에 되지 않는다함.다리가 가볍기 때문에 모터가 토크를 줬을 때 지체 없이 가속(High acceleration)할 수 있어 동적인 움직임이 가능해짐.



### 시스템 구조

![System architecture](/assets/images/posts/mit-cheetah3/cheetah_architecture.png)


A. 입력 및 계획 (Green Box: High-Level Planning)
- 사용자 명령: 조이스틱으로 줄수 있는 수준의 $$\dot{p}_d$$ (목표 속도)와 $$\dot{\psi}_d$$ (회전 속도) 단순 명령만 받음 
- Gait Scheduler (보행 스케줄러): 시간($$t$$)에 따라 다리가 'Swing(공중)' 상태인지 'Stance(지지)' 상태인지 계획을 설계($$s_{\phi}$$).
- Desired CoM State: 사용자의 명령과 스케줄러를 합쳐서 "몸통이 미래에 어디에 있어야 하는지" 궤적을 그림

B. 제어의 분기점 (Orange Box: Leg Control)
  - Swing Leg Control: 다리가 공중에 있을 때는 목표 위치로 발을 옮기는 **위치 제어(Position Control)**
  - Force Control (MPC/QP): 다리가 땅에 닿아 있을 때는 몸통의 균형을 잡기 위해 지면을 미는 **힘 제어(Force Control)**
- Switching: 이 두 제어 모드를 오가는 스위치 역할을 하는 것이 바로 **$$s_{\phi}$$(Contact State)**임

C. 피드백 및 상태 추정 (blue Box: State Estimation)
- **CoM State KF**(위치/속도 추정)와 **Leg Contact Detection**(접촉 감지) 블록이 현재 로봇의 상태를 추정하여 다시 컨트롤러로 피드백을 보냄


로봇의 다리가 땅에 닿았는지, 공중인지에 따라 다른 제어기를 사용하므로 다리가 땅에 닿았는지를 판단할수있는 근거인 Leg Contact Detection 을 제일 먼저 구현해보려 한다.

 
## MuJoCo Menagerie

 MuJoCo Menagerie는 DeepMind에서 공개한 로봇 시뮬레이션 모델 컬렉션으로, MuJoCo 물리 엔진 기반의 로보틱스 연구를 목적으로 제작되었다. 실제 로봇과 유사한 기구·질량 특성을 가진 표준화된 모델을 제공해, 하드웨어 없이도 제어 알고리즘을 실험하고 비교할 수 있도록 한다. 본 프로젝트에서는 이 중 Unitree Go2 모델을 사용할 예정.

 [MuJoCo Menagerie Link](https://github.com/google-deepmind/mujoco_menagerie)

 ![MuJoCo Menagerie](/assets/images/posts/mit-cheetah3/mujoco.png)