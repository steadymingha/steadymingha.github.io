---
title: "Robot Arm Machine Tending with Diffusion Policy 1"
tagline: "Learning a wirebonding magazine-loading task in simulation"
excerpt: "CBiRRT expert demonstrations and dataset construction for a robot-arm pick-and-place task"
categories:
  - Manipulation
tags:
  - diffusion policy
  - manipulation
  - imitation learning
author_profile: false
header:
  image: /assets/images/header/robot_arm_1200x600.png
  teaser: /assets/images/header/robot_arm_1200x600.png
---



## 배경

본 프로젝트는 클린룸 내 와이어 본딩(Wirebonding) 작업을 수행하는 로봇팔의 모션 플래닝 시뮬레이션 환경을 활용하고자 시작되었다. 기존 Pick and Place 작업에 사용된 CBiRRT 알고리즘은 목표 궤적을 개루프(Open-loop) 방식으로 계산하여 실행한다. 때문에 AGV 위치 오차와 인식(Perception) 노이즈를 보정하기 위해, 매번 Aruco Tag를 촬영하여 타겟 위치와 포즈를 재정렬해야 하는 번거로운 작업이 동반된다.
현재 실물 로봇은 YOLOx 모델로 매거진을 탐지한 후 CBiRRT를 이용해 접근하는 방식을 취하고 있어서, 로봇의 접근 과정을 자동화하고 번거로운 Tag 인식 단계까지 생략할 수 있는 모방 학습(Imitation Learning)이 더 효율적일 것이라 판단. 이에 따라, 모방 학습 분야에서 가장 뛰어난 성능을 보이는 알고리즘 중 하나인 Diffusion Policy를 이번 Task에 시뮬레이션으로 시범 적용해보기로 한다. 
 


<div class="sl"></div>



## 시뮬레이션 환경 구축

사용할 로봇팔은 Dobot CR7V로 결정되었고, 이 로봇팔은 Dobot ROS2 SDK로 github에 공개되어있다. 개발 환경이 Gazebo 기준으로 구성되어 있어 로봇팔이 작업해야할 선반, Wirebonding 머신, 매거진, 그리퍼 등 필요한 모든 환경과 모델을 URDF로 제작했다. 그리퍼 형태를 정하고, 공장에 설치된 장비와 선반을 실측하여 Blender로 mesh를 만들었다. 충돌 회피 계산에 반영되어야해서 collision mesh를 Primitive Collider로 구성했다.


![alt text](/assets/images/posts/diffusionpolicy/gripper_blender.png)


![alt text](/assets/images/posts/diffusionpolicy/gripper_gazebo.png)
<i>*L자로 고정 jaw가 매거진을 받치고, moving jaw가 슬라이딩으로 잡도록 구성</i>

<div class="img-row" markdown="1">
![alt text](/assets/images/posts/diffusionpolicy/wbmachine1.png)

![alt text](/assets/images/posts/diffusionpolicy/wbmachine2.png)
</div>

![alt text](/assets/images/posts/diffusionpolicy/wbmachine3.png)
<i>*로봇팔이 작업할 KnS사의 Wirebonding 머신 실물(상)과 Primitive 모델링(하)</i>

<div class="sl"></div>

## Task 정의

본 시리즈에서 구현될 작업은 Wirebonding 공정에 투입되기 전 단계로, 매거진을 선반에서 가져와 로봇 베이스에 싣는 시뮬레이션 작업이다. 샘플링 기반 기법으로 경로 계획을 수행할 때 충돌 검사 부담을 줄이기 위해, 전체 경로를 한 번에 계획하지 않고 다음과 같이 단계별로 분할하여 설계했다. 

> 매거진 탐지 -> 매거진 위로 접근 -> 그리퍼(J6 관절) 회전 -> 하강 -> 매거진 파지 -> 상승 -> 접근 경로 역순 복귀

로봇팔은 실제 사용할 팔 모델인 DOBOT CR7V URDF 모델을 이용하였으며 AGV는 Neobotix의 MPO-700 ROS2 오픈소스를 사용했다. 기존 룰 기반 Pick and Place 시뮬레이션은 제공된 로봇팔 SDK에 맞춰 Gazebo 기반으로 개발되었으나, Diffusion Policy 학습 데이터 수집을 위해 Isaac Sim으로 포팅하였다.

![alt text](/assets/images/posts/diffusionpolicy/cleanroom1.png)

<div class="sl"></div>
<div class="sl"></div>

## 관측 및 행동 공간 설계 (Observation & Action Space)

Diffusion Policy 적용을 위한 Observation 데이터로는 두 종류의 카메라 뷰를 사용하였다. eye-in-hand 카메라는 그리퍼에 부착되어 매거진에 접근하는 과정을 근접 시점에서 관측하고, scene 카메라는 로봇과 매거진을 함께 담는 전역 시점을 제공한다.

| 관측값 | 형태 | 내용 |
| :--- | :--- | :--- |
| agentview_image | (240, 320, 3) uint8 | 고정 카메라. AGV 베이스에 달려 있어서 정차 위치에 따라 시야가 달라짐 |
| robot0_eye_in_hand_image | (240, 320, 3) uint8 | D405 손목 카메라 |
| robot_eef_pose | 7D: xyz + quat | 로봇팔 끝단 (eef) 자세, quat은 6D rotation representation으로 변환 |
| gripper | 1D | 그리퍼 명령값, 열림/닫힘 |

| 행동값 | 형태 | 내용 |
| :--- | :--- | :--- |
| action | (T, 8) float32 | [0:3] 위치 xyz(m) · [3:7] 회전 quat xyzw · [7] 그리퍼 명령 |

<div class="sl"></div>
<div class="sl"></div>

## 전문가 시연 데이터 수집

Diffusion Policy 학습에 사용할 전문가 시연 데이터는 사람의 직접 조작 대신 CBiRRT 플래너가 생성한 궤적으로 대체하였다. 매거진 파지부터 적재까지의 시퀀스를 사전에 구축해두고, 이를 시뮬레이션 상에서 반복 실행하는 스크립트를 통해 자동으로 수집하였다.
매거진을 선반 1,2층에 동일한 간격으로 배치하고 
학습 257 , 평가 5로 총 262 에피소드로 제작하였다. 데이터 수집 과정에서는 분포의 균형, 관측 조건의 다양성, 데이터 품질을 각각 고려하였다.

**분포 균형** — 픽 대상은 매 패스마다 균일 랜덤 순열로 결정된다. 선반 1단의 10개 박스를 무작위 순서로 한 번씩 처리하므로 특정 위치의 샘플이 과대 표집되지 않는다.

**관측 조건의 다양성** — 위 방식의 결과로 선반이 가득 찬 상태부터 거의 비워진 상태까지 모든 재고 상황이 데이터에 포함된다. 여기에 2단 박스를 각각 50% 확률로 제거하여 배경의 시각적 변화를 추가했고, AGV 주차 위치에는 대상 박스가 속한 구역의 중심을 기준으로 x축 ±0.05m, y축 ±0.03m의 랜덤 오프셋을 적용했다. 동일한 박스라도 매번 다른 상대 위치에서 관측되므로, 정책이 고정된 좌표를 외우는 대신 관측에 조건부로 반응하도록 유도한다.

**데이터 품질** — 저장은 성공한 에피소드에 한정한다. 매거진이 포켓에 정상 안착했는지 확인하고, 주변 박스가 허용 범위(2~3cm) 이상 움직였는지를 검사하여 이웃을 건드린 시도는 폐기하였다. 


<div style="max-width: 600px; margin: 0 auto;">
  <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden;">
    <iframe
      src="https://www.youtube.com/embed/uVWRJd4w6qI?si=EKn79on31lySUj6b"
      style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; border: 0;"
      allowfullscreen>
    </iframe>
  </div>
</div>

<div class="sl"></div>
<div class="sl"></div>

## 학습 결과
영상은 600 에폭 계획 중 230 에폭 시점의 체크포인트로 수행한 롤아웃이다. 20회 롤아웃 중 성공은 3회였고, 실패의 대부분은 정책이 생성한 action 청크의 IK 해를 찾지 못해 실행이 막힌다. 아직 정책이 작업을 학습했다고 보기는 어려운 상태로, 우선 600 에폭까지 학습을 마친 뒤 성능을 재확인하고, IK 실패의 원인이 데이터 부족인지 action 표현 방식인지 파악할 계획이다.

<div style="max-width: 600px; margin: 0 auto;">
  <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden;">
    <iframe
      src="https://www.youtube.com/embed/qbg6uWm5XIk?si=BU1zF2ePZVzQ1yYC&amp;start=3"
      style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; border: 0;"
      allowfullscreen>
    </iframe>
  </div>
</div>

<div class="sl"></div>

## Reference

***[1] C. Chi et al., “Diffusion Policy: Visuomotor Policy Learning via Action Diffusion,” (2023)*** 

