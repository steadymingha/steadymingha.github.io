---
title: "Unitree G1 Takes on a TikTok Dance Challenge"
tagline: "Motion tracking from human dance videos to a simulated humanoid"
excerpt: "Teaching a Unitree G1 humanoid a TikTok dance challenge with BeyondMimic"
categories:
  - Locomotion
tags:
  - beyondmimic
  - humanoid
  - unitree g1
  - motion tracking
  - reinforcement learning
author_profile: false
header:
  image: /assets/images/header/dancingbot2.png
  teaser: /assets/images/header/dancingbot.png
---


## 배경

요즘 릴스에 자주 뜨는 댄스 챌린지 영상들을 보다가 문득 휴머노이드가 얼마나 빠르고 정확하게 이 춤을 배울수 있을지 궁금해졌다. 실물로봇으로는 물리적 제약이 많고 쉽지않을테니 시뮬레이션상에서 학습 알고리즘으로 빠르게 성능을 낼 수 있는지 알아보기로. 관련 기술을 찾아보니 BeyondMimic이라는 모션 모방 프레임워크가 있었고, 이를 활용해 로봇에게 직접 내가 원하는 춤을 가르쳐보기로 한다.


<div class="sl"></div>

## 학습 모델 — BeyondMimic

<div class="ss"></div>

<div style="max-width: 600px; margin: 0 auto;">
  <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden;">
<iframe id="g1-loop" width="560" height="315" src="https://www.youtube.com/embed/RS_MtKVIAzY?enablejsapi=1&autoplay=1&mute=1&playsinline=1" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
  </div>
</div>
<script src="https://www.youtube.com/iframe_api"></script>
<script>
  // 0~4초 구간만 반복
  function onYouTubeIframeAPIReady() {
    var player = new YT.Player('g1-loop', {
      events: { onReady: function () {
        setInterval(function () {
          if (player.getCurrentTime() >= 4) player.seekTo(0, true);
        }, 100);
      } }
    });
  }
</script>

<div class="sm"></div>

BeyondMimic은 인간의 동작을 휴머노이드 로봇이 따라 하도록 학습하는 프레임워크다. 몸체의 위치·자세·속도가 레퍼런스와 얼마나 일치하는지를 보상으로 사용하며, 다양한 고난도 동작을 공통된 보상 구성과 하이퍼파라미터로 학습할 수 있다는 점이 특징이다.

크게는 동작을 따라 배우는 **모션 트래킹** 단계와, 배운 동작들을 증류한 모델을 통해 추가 학습 없이 새로운 목표에 맞는 동작을 만들어내는 **guided diffusion** 단계로 구성된다. 이번 실험에서는 이 중 모션 트래킹을 활용해, 내가 고른 춤을 G1이 따라 하도록 학습시켰다.


<div class="sl"></div>

## 첫번째 기술 습득

먼저 HuggingFace의 LAFAN1 G1 변환 데이터에서 dance1_subject1(2분 11초)을 받아서 mjlab에서 학습을 진행했다. dance1_subject는 아래와 같은 기본적인 춤동작에 대한 뼈대 데이터다:


<div style="max-width: 600px; margin: 0 auto;">
  <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden;">
    <iframe
      src="https://www.youtube.com/embed/Ydp5W3J3-nA?si=dw-KKDy22xSsIHBU&amp;start=38"
      style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; border: 0;"
      allowfullscreen>
    </iframe>
  </div>
</div>

<div class="sm"></div>

dance1 학습 결과는 30,000 iteration, 약 6시간정도 걸렸고 위 데이터셋 영상과 거의 유사하게 춤을 춘다. 여기에 EmetSound의 ***"GUAP 챌린지"***를 시켜보기로 했다. 반복동작이라 금방 따라할 것으로 봤다.

<div class="sl"></div>

## 두번째 학습

유튜브 튜토리얼에서 9초짜리 춤 구간을 잘라냈다.
 <i>(출처 : [EmetSound GUAP Tutorial](https://youtu.be/0iNVsMovE2E?t=712)) </i>

<div class="sm"></div>

![Reference dance](/assets/images/posts/beyondmimic/g1_dance.gif)

여기서 영상 기반 3D 휴먼 모션 추정 모델(GVHMR)로 영상속 사람을 SMPL형태로 추출했다.
SMPL 은 사람의 몸을 자세파라미터와 체형파라미터로 표현하는 표준 인체 모델로 막스플랑크 연구소에서 만들었다함. 이걸 GMR(General Motion Retargeting)으로 사람 관절값을 G1 관절값으로 리타게팅해주고 BeyondMimic 입력에 맞춘다.

<div class="sm"></div>

<img src="/assets/images/posts/beyondmimic/flowchart.png" alt="Flow chart" style="width: 100%; display: block; margin: 0 auto;">

<div class="sm"></div>

<!-- 내가 고른 춤 데이터 만들기 (괍 튜토리얼, 9초)
- 유튜브 튜토리얼에서 전체 춤 구간만 잘라냈어요 (세 명 중 가운데 사람이 자동 선택됨).
- SMPL과 SMPL-X 몸 모델을 받았어요 (공식 사이트 가입 필요).
- GVHMR로 영상에서 사람 3D 동작을 추출했어요. 고정 카메라라 DPVO는 건너뛰었어요. 학습된 모델 파일은 Google Drive 다운로드 한도에 걸려서 HuggingFace 미러에서 받았어요.
- GMR로 사람 동작을 G1 관절로 변환한 다음 CSV(36열, 30fps)로 저장했어요.
- 학습 전에 관절 각도와 속도 한계를 점검했어요. 가장 빠른 관절도 속도 한계의 45%였어요.
- mjlab의 csv_to_npz로 학습용 npz(50fps)로 변환했어요. -->

첫 학습에 이어서 Fine-tuning 했고, 10,000 iteration을 약 두시간동안 돌렸다. 결과:


<div style="max-width: 600px; margin: 0 auto;">
  <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden;">
    <iframe
      src="https://www.youtube.com/embed/WZ1CuHzq_9g?si=WFezPp2-C6yyQE2j"
      style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; border: 0;"
      allowfullscreen>
    </iframe>
  </div>
</div>

<div class="sl"></div>


## 세번째 학습

좀더 어려운 춤을 가르쳐보기로 했다. 전부터 배워보고 싶었던 Scott Forsyth의 ***"I want you back challenge"***. 데이터셋 만드는 방식은 위와 동일하고, I Want You Back(15.7초)을 이전 춤(GUAP) 정책에서 이어서 15,000 iteration 학습시켰다. <i>(출처 : [블레이즈 VLAZE 댄스 튜토리얼](https://www.youtube.com/watch?v=eAboOdRFgLU&t=14s))</i>

<div class="sl"></div>

![Reference dance](/assets/images/posts/beyondmimic/noturn.gif)

<div class="sl"></div>

어설프게 따라는 하는데, 회전구간만 쏙 빼놓고 한다(잔상이 로봇 레퍼런스 동작). 회전구간이 적으니 연습이 더 필요한가? 회전만 부분연습을 더 시켜보기로 했다. 소용이 없다. 회전을 건너뛰면 넘어지질 않으니 에피소드 종료가 일어나질 않아 추가연습을 시켜봤자 학습이 안되었던 것이다. 회전을 건너뛰면 작은 보상만 잃지만 회전을 시도하다 넘어지면 에피소드가 끝나버려 이후의 보상을 전부 잃는 구조였다. 회전을 시도하도록 보상을 다시 설계했다.

<div class="sl"></div>


| 수정                     | 기존            | 변경                                                                        | 의도                                  |
| :--------------------- | :------------ | :------------------------------------------------------------------------ | :---------------------------------- |
| (1)몸통 방향 보상 가중치        | 0.5           | 3.0                                                                       | 회전을 건너뛰는 손해를 키움                     |
| (2)몸통 방향 허용 폭          | 0.4 rad (23°) | $1.0 \mathrm{rad}\left(57^{\circ}\right)$                                 | 반쯤 돌아도 점수가 나오게 해서, 점점 더 돌도록 유도      |
| (3)회전 속도 보상 (새로 추가)    | 없음            | 몸통 수평 회전 속도가 레퍼런스와 비슷하면 보상 (가중치 2.0 , std $4 \mathrm{rad} / \mathrm{s}$ ) | 맞는 방향으로 돌기 시작만 해도 득점              |
| (4)수평 방향 종료 조건 (새로 추가) | 없음            | 레퍼런스와 $90^{\circ}$ 이상 어긋나면 에피소드 종료                                        | 회전을 건너뛰면 바로 끝나서, 시도하는 것 외에 방법이 없게 함 |

<div class="sl"></div>

회전을 못 배운 전체 춤 정책에서 시작해서, 같은 설정으로 3,000 iteration(약 40분)만 이어서 학습했고 결과는 회전까지 하면서 어느 정도 춤을 잘 따라하게 됐다. 

<div class="sm"></div>

<div style="max-width: 600px; margin: 0 auto;">
  <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden;">
    <iframe
      src="https://www.youtube.com/embed/D4LcjdockPY?si=0KyGfYU6eHK9EdMG"
      style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; border: 0;"
      allowfullscreen>
    </iframe>
  </div>
</div>

<div class="sl"></div>

## 마무리

3D 모션 추정, 로봇관절 리타게팅, 모션 트래킹 학습을 하기까지 거치는 단계가 많아 레퍼런스 동작 자체의 깨끗함이 좀 떨어지는 듯하다. 학습한 정책이 구현한 동작도 디테일이 좀 떨어지지만 이것도 보상설계를 정교하게 더 정교하게 한다면 좀 더 나아지지 않을까 한다. 

<div class="sl"></div>



## Reference

***[1] Q. Liao et al., “BeyondMimic: From Motion Tracking to Versatile Humanoid Control via Guided Diffusion,” (2025)***

***[2] X. B. Peng et al., “DeepMimic: Example-Guided Deep Reinforcement Learning of Physics-Based Character Skills,” (2018)***

***[3] F. Harvey et al., “Robust Motion In-betweening,” (2020)*** — LAFAN1 dataset

***[4] Z. Shen et al., “World-Grounded Human Motion Recovery via Gravity-View Coordinates,” (2024)*** — GVHMR

***[5] G. Pavlakos et al., “Expressive Body Capture: 3D Hands, Face, and Body from a Single Image,” (2019)*** — SMPL-X

***[6] J. P. Araujo et al., “Retargeting Matters: General Motion Retargeting for Humanoid Motion Tracking,” (2025)*** — GMR
