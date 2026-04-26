# Resamp 지식 요약 (CQ MCP 등록용)

## 프로젝트 개요
- 이름: `Resamp`
- 목적: UTAU/OpenUtau용 WORLD 기반 리샘플러.
- 핵심 파이프라인: CLI 파싱 -> 소스 트리밍 -> WORLD 분석 -> 타겟 F0 매핑 -> WORLD 렌더 -> 볼륨/페이드 후처리 -> WAV 저장.

## 핵심 DSP 구조
- 분석기: DIO + StoneMask + CheapTrick + D4C.
- 분석 프레임 주기: 2.5ms.
- CheapTrick 스무딩: `q1 = -0.30`.
- 렌더 프레임 주기: 적응형
  - 변조 노트: `0.50ms`
  - 일반 노트: `0.65ms`
  - 평탄 노트: `0.80ms`
- 연결 처리:
  - 자음/연결부 1회 통과
  - 안정 모음 구간 루프
  - 루프 경계 seam 크로스페이드
  - 자음/연결부 join 스무딩 추가

## 피치 처리
- `pitch_bend`는 arg[13]에서 디코드됨 (12-bit + RLE 또는 legacy int8 base64).
- 샘플 단위 cents contour를 zero-phase FIR로 스무딩 후 F0에 적용.
- 전역 cents 플래그:
  - `t` = 피치 cents 오프셋 (`-1200..1200`).

## 현재 플래그 모델
- 톤/발성:
  - `g` 젠더/포먼트 워프
  - `Bi` 밝기
  - `Hu` 허스키 톤
  - `Mo` 입 열림
  - `Tn` 텐션 (기존 `t`에서 이름 변경)
  - `Gr` 그로울
  - `c` 보이스 컬러
- 배음/노이즈:
  - `H/Hr` 배음량 (중립 70)
  - `N` 노이즈 양
  - `Bh` 브레스니스
  - `Ns` 노이즈 컬러
- 연결/아티큘레이션:
  - `Tr` 연결 길이
  - `Cs` 자음 안정화
  - `At` 어택
  - `Rl` 릴리즈 에어
- 다이내믹:
  - `P` 피크 컴프레션
- 성도 시뮬레이터 레이어:
  - `Vtl` 성도 길이
  - `Vtr` 공명 중심 이동
  - `Vtw` 공명 폭/포커스
  - `Vc` 협착
  - `Nn` 비성 결합

## 성도 시뮬레이터 레이어 동작
- WORLD의 스펙트럼/AP shaping 단계 내부에서 동작.
- `Vtl/Vtr/Vtw/Vc/Nn`에 `Mo`를 결합해 다음을 수행:
  - 추가 tract warp (`Vtl`)
  - 다중 포먼트 중심/포커스 이동 (`Vtr/Vtw`)
  - 협착 강조 (`Vc`)
  - 비성 포먼트 + 노치 특성 (`Nn`)
  - 프레임 에너지 정규화(볼륨 드리프트 완화)
- 최근 튜닝:
  - 중간값 체감 강도 상향(응답 커브/가중치/drive 강화)
  - `Vtl/Vtr/Vtw` ±방향 변화가 더 명확하게 들리도록 조정
  - 극단값(±100)에서 변화량이 분명하게 들리도록 워프/포먼트 이동량/스펙트럼 가중치를 추가 상향
  - 과구동으로 인한 파열/노이즈 방지를 위해 tract layer에 soft-limit + mix cap 적용
  - 공통 drive/mix 바닥값을 낮춰 파라미터 값별 반응 차이를 복원
  - AP(노이즈) 연동을 크게 축소해 활성화 시 음성 마스킹 문제를 완화
  - `Vc/Nn`은 `0..100` 범위만 지원(음수 입력 시 0으로 클램프)

## 후처리
- 적응형 라우드니스 정규화 (`RMS`, `p95`, `p99.5`).
- 텐션 연동 리미팅.
- 최종 hard peak guard (`p99.9` 기반).
- 짧은 fade in/out 적용.

## OpenUtau 배포 동작
- `resamp` 빌드 후 자동 복사:
  - `C:/Users/oyh57/SODAsoo1/VocalSynth/OpenUtauV-win-x64/Resamplers/resamp.exe`
  - `C:/Users/oyh57/SODAsoo1/VocalSynth/OpenUtauV-win-x64/Resamplers/resamp.yaml`
- `wavtool_probe` 빌드 후 자동 복사:
  - `C:/Users/oyh57/SODAsoo1/VocalSynth/OpenUtauV-win-x64/Resamplers/wavtool_probe.exe`

## Wavtool M0 (인터페이스 캡처)
- 목적: OpenUtau 외부 wavtool 호출 인자 계약 캡처.
- 실행 파일: `wavtool_probe.exe`
- 로그: 기본 `%TEMP%/resamp_wavtool_calls.jsonl` (`RESAMP_WAVTOOL_LOG`로 변경 가능)
- 참고 문서: [WAVTOOL_M0.md](/C:/Users/oyh57/SODAsoo1/Devs/Resamp/WAVTOOL_M0.md)

## 운영 참고
- 밝기 `B` alias는 제거되었고 `Bi`만 사용.
- `modulation` 인수는 인위적 LFO 비브라토 합성에 사용하지 않음.
- 외부 Classic resampler 경로는 호출 1회당 플래그 1세트만 전달됨(음소 내부 연속 커브 자동화 없음).
