# Resamp GMM 기반 음색 변조 설계안 (v0.1)

이 문서는 현재 `Resamp`(WORLD 기반 외부 resampler)에 **고전 ML(GMM)** 을 도입해
음성 변조의 사실감과 파라미터 분리감을 높이기 위한 실구현 설계안입니다.

## 1) 목표 / 비목표

목표:
- `Vtr/Vtw/Vc/Nn/Mo/Hu/Tn/Bh/N` 플래그 변화가 더 자연스럽고 일관되게 들리게 만들기
- 극단값에서 파열/노이즈를 줄이고, 중간값에서 체감 차이를 확보하기
- 현재 C++/WORLD 파이프라인에 무리 없이 통합하기

비목표:
- HSMM 기반 duration/state 생성기 구현 (외부 resampler 호출 구조와 부적합)
- end-to-end 신경망 보코더로 전면 교체

## 2) 왜 HSMM이 아니라 GMM인가

- 현재 문제는 주로 **스펙트럼/노이즈 질감 변환 품질** 문제이며, HSMM의 주특기(상태 지속시간/trajectory 생성)와 직접적으로 맞지 않습니다.
- OpenUtau 외부 resampler는 기본적으로 노트 단위 호출이라, HSMM 장점(긴 시퀀스 문맥)이 제한됩니다.
- GMM은 프레임 단위 조건부 변환에 적합하고, 기존 WORLD feature(`sp/ap/f0`)와 바로 연결됩니다.

## 3) 적용 전략 (권장)

권장 전략: **Delta-Mapping GMM**

- 입력 `x`: 현재 프레임 음향 feature + 플래그 조건
- 출력 `y`: 규칙기반 변환 결과 대비 보정량(delta)
- 최종 적용: `feature_out = feature_rule + y_hat`

핵심:
- 기존 규칙기반(현재 tract layer)을 완전히 버리지 않고, **GMM을 보정 레이어**로 사용
- 기존 사운드 성격 유지 + 데이터 기반 자연스러움 보강
- 장애 시 즉시 bypass 가능

## 4) 피처 설계

프레임 단위 feature:
- `mcep[24]` (CheapTrick에서 추출한 envelope를 mel-cepstrum으로 변환)
- `bap[5]` (D4C 기반 band aperiodicity 요약)
- `logf0` (voiced만, unvoiced는 0 + 마스크)
- `uv` (0/1 voiced flag)
- 위치 컨텍스트: 노트 상대시간 `t_rel` (0..1), onset/release 구간 마스크
- 플래그 조건 벡터:
  - `Vtl,Vtr,Vtw,Vc,Nn,Mo,Hu,Tn,Bh,N` (정규화 -1..1 또는 0..1)

권장 출력:
- `delta_mcep[24]`
- `delta_bap[5]`
- (선택) `delta_logf0`는 초기에 비활성 권장  
  이유: 현재 피치 아티팩트 이슈 재발 위험

## 5) 모델 구조

모델 1개(초기):
- 전역 Full-cov GMM 또는 Diag-cov GMM
- mixture 수: `M=16`부터 시작 (`8/16/32` A/B)
- joint vector: `z = [x, y]`

추론식(조건부 평균):
- 각 mixture `m`에 대해  
  `E[y|x,m] = mu_y_m + Sigma_yx_m * inv(Sigma_xx_m) * (x - mu_x_m)`
- posterior `gamma_m = p(m|x)`
- `y_hat = sum_m gamma_m * E[y|x,m)`

안정화:
- covariance floor
- 출력 delta clip (파라미터별 상한)
- voiced/unvoiced 분리된 소형 GMM 2개로 나누는 옵션

## 6) 학습 데이터 구성

데이터 소스:
- 동일 voicebank/동일 가사에서 플래그 세트만 다르게 렌더한 페어
- 기본(중립) 렌더 + 스타일 렌더를 쌍으로 구성

추천 플래그 샘플링:
- 라틴 하이퍼큐브 또는 균등 랜덤
- 과도한 조합(모든 플래그 극단 동시)은 비율 제한

정렬:
- 같은 score/lyric 기준 렌더여도 미세 시간차가 있으므로 frame DTW 또는 forced index align 사용

데이터 규모(초기):
- 30~60곡 분량, 총 voiced 프레임 200k 이상 권장

## 7) 런타임 통합 위치

현재 파이프라인 기준:
1. WORLD 분석
2. 기존 규칙기반 변조(tract/tension/etc.)
3. **GMM delta 보정 레이어 적용**  ← 신규
4. 에너지 정규화/클리핑 가드
5. WORLD synthesis

코드 삽입 후보:
- `src/synth/world_synth.cpp` 내 frame loop (현재 tract layer 이후, 후처리 이전)

## 8) 설정/파일 포맷

신규 파일:
- `models/gmm_voice_v1.bin` (모델 파라미터)
- `models/gmm_voice_v1.meta.json` (feature order, mean/std, mixture 수, 버전)

신규 플래그/옵션(제안):
- `Gm` (`0/1`): GMM 레이어 on/off
- `GmS` (`0..100`): GMM blend 강도
- 환경변수 `RESAMP_GMM_MODEL=<path>`

## 9) 성능 예산

목표:
- 현재 대비 렌더 시간 +15% 이내

방법:
- mixture 수 제한 (`<=16`)
- 프레임별 반복 계산에서 matrix inverse 사전 계산
- SIMD 가능한 루프 우선

## 10) 검증 계획

객관 지표:
- MCD (mel-cepstral distortion)
- BAP RMSE
- 프레임 에너지 변동성(노트 간 loudness 편차)

청감 A/B:
- 파라미터 독립성: 한 번에 1개 플래그만 ±변화
- 극단값 안정성: 파열/노이즈/금속성 발생 여부
- CVVC 연결부 자연스러움

합격 기준(초기):
- MCD 개선 + 청감 블라인드 70% 이상 선호
- 아티팩트 증가 없음

## 11) 단계별 구현 로드맵

Phase 1 (빠른 프로토타입):
- feature dump 도구 추가 (mcep/bap/logf0/flags)
- Python 학습 스크립트로 GMM 학습
- C++ 런타임 로더 + inference + blend

Phase 2 (안정화):
- voiced/unvoiced 분리 모델
- mixture/정규화/clip 튜닝
- 실패 시 안전 bypass 경로 정리

Phase 3 (고도화):
- 음소군(모음/자음/비음)별 서브모델
- wavtool 연계 구간 단위 smoothing

## 12) 리스크와 대응

리스크:
- 데이터 편향 -> 특정 음소에서 과보정
- 극단값에서 과도한 delta
- 런타임 증가

대응:
- 음소군별 샘플 균형
- delta clip + 에너지 가드
- `GmS` 강도 및 즉시 off 스위치 제공

---

요약:
- 이 프로젝트에는 HSMM보다 **GMM delta 보정 레이어**가 현실적입니다.
- 기존 규칙기반을 유지한 채 GMM으로 자연스러움/분리감을 올리는 하이브리드 방식이 구현 난이도, 안정성, 롤백 용이성에서 가장 유리합니다.
