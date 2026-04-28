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
- CLI 파서 보강:
  - `flags` 인자가 생략되어 optional 인자가 당겨지는 호출 형태를 자동 보정.
  - 결과적으로 노트별 플래그 누적/오인식 가능성을 줄임.

## 현재 플래그 모델
- 톤/발성:
  - `g` 젠더/포먼트 워프
  - `Bi` 밝기
  - `Hu` 허스키/브라이트 톤 (`+` 밝음, `-` 허스키)
  - `Mo` 입 열림(+)/입 닫힘(-)
  - `Tn` 텐션 (기존 `t`에서 이름 변경, 현재는 성도 시뮬레이터 레이어에도 직접 결합)
  - `Gr` 그로울
  - `c` 보이스 컬러
- 배음/노이즈:
  - `H/Hr` 배음량 (중립 70)
  - `N` 노이즈 양 (+추가/-억제)
  - `Bh` 브레스니스 (+추가/-억제)
  - `Ns` 노이즈 컬러
- 연결/아티큘레이션:
  - `Cs` 자음 안정화
  - `At` 어택
- 다이내믹:
  - `P` 피크 컴프레션
  - `Ln` 음량 정규화 강도 (`+` 균일화 강화 / `-` 정규화 완화)
- 성도 시뮬레이터 레이어:
  - `Vtl` 성도 길이
  - `Vtr` 공명 중심 이동
  - `Vtw` 공명 폭/포커스
  - `Vc` 협착
  - `Nn` 비성 결합/억제 (`+` 비성 증가, `-` 비성 억제)

## 성도 시뮬레이터 레이어 동작
- WORLD의 스펙트럼/AP shaping 단계 내부에서 동작.
- `Vtl/Vtr/Vtw/Vc/Nn`에 `Mo`를 결합해 다음을 수행:
  - 추가 tract warp (`Vtl`)
  - 다중 포먼트 중심/포커스 이동 (`Vtr/Vtw`)
  - 협착 강조 (`Vc`)
  - 비성 포먼트 + 노치 특성 (`Nn`, 음수에서는 비성 억제)
  - 프레임 에너지 정규화(볼륨 드리프트 완화)
- 최근 튜닝:
  - 중간값 체감 강도 상향(응답 커브/가중치/drive 강화)
  - `Vtl/Vtr/Vtw` ±방향 변화가 더 명확하게 들리도록 조정
  - 극단값(±100)에서 변화량이 분명하게 들리도록 워프/포먼트 이동량/스펙트럼 가중치를 추가 상향
  - 과구동으로 인한 파열/노이즈 방지를 위해 tract layer에 soft-limit + mix cap 적용
  - 공통 drive/mix 바닥값을 낮춰 파라미터 값별 반응 차이를 복원
  - AP(노이즈) 연동을 크게 축소해 활성화 시 음성 마스킹 문제를 완화
  - 최신 튜닝에서 `Vtr/Vtw/Vc/Nn` 입력 감도(power curve)와 wet mix를 추가 상향해 중간값에서도 변화가 더 빨리 들리도록 조정
  - 최신 튜닝에서 `Vtr/Vtw/Vc/Nn` 강도를 한 단계 더 증폭(포먼트 이동량/대역 가중치 상향)하고, 에너지 정규화 적용률을 낮춰 캐릭터 차이가 더 분명하게 남도록 조정
  - 최신 튜닝에서 `Vtr/Vtw/Vc/Nn` 스펙트럼 변조 강도는 높이되 AP 증분은 분리/제한해 노이즈 폭주 없이 캐릭터 차이를 확보
  - 최신 튜닝에서 성도 레이어 결합을 공통 dB 합산에서 모듈별 순차 blend 방식으로 변경(Length/Resonance/Width/Constriction/Nasal 분리도 향상)
  - 정규화는 항상 적용하지 않고, 프레임 에너지 드리프트가 큰 경우에만 약보정하도록 변경
  - 최신 튜닝에서 `Vtw/Vc/Nn`에 최소 mix 바닥값과 대역 가중 강화(저값 체감 향상), 대신 AP 증분은 더 줄여 노이즈 증가는 억제
  - `Vtr/Vtw/Vc/Nn` 파라미터의 스펙트럼 가중치/중심대역을 재튜닝해 체감 강도 상향
  - VocalTractLab 방식에 맞춰 공명/반공명(resonance/anti-resonance) 분리 근사 모델로 `Vtr/Vtw/Vc/Nn` 변조를 재구성
  - 정규화가 변조 차이를 과하게 상쇄하지 않도록 부분 정규화 적용
  - `Vtr/Vtw/Vc/Nn`은 공통 drive 의존을 줄이고 파라미터별 독립 강도 스케일/리미터로 분리 적용
  - `Mo`는 양/음수 분리 제어(음수 시 입 닫힘/포먼트 하향 성향 강화)
  - `Mo`는 전용 formant warp(`mo_ratio`, `mo_blend`) + 밴드별 재가중치를 추가해 개방/닫힘 체감 차이를 확대
  - `Gr`(그로울) 효과 강도 추가 상향
  - `Bh`/`N`은 음수 입력 시 각각 숨소리/노이즈 억제 동작 추가
  - `Vc`는 `0..100`, `Nn`은 `-100..100`을 지원
  - 최신 빌드에서 `Vtl+`와 `Vtw` 체감 강도를 올리고, 전역 바람/팝 노이즈 가드를 추가
  - 최신 빌드에서 `H/Hr`를 70보다 조금 낮춘 구간의 과도한 거칠기를 완화
  - 최신 빌드에서 `Hu-`는 약한 포먼트 하향과 제한된 rasp를 더해 낮고 쉰 듯한 인상을 강화
  - 최신 빌드에서 THROAT식 성도 플래그(`Vtl/Vtr/Vtw/Vc/Nn/Mo`)가 활성화될 때만 5구간 tube-response 곡선을 추가 적용
  - 최신 빌드에서 `Vtr/Vc/Nn` 모듈 강도 및 wet mix를 추가 상향해 체감 반응을 강화
  - 최신 빌드에서 성도 레이어 정규화 임계/보정량을 조정해 강도 상승 시 볼륨 요동을 완화
  - 최신 빌드에서 `Ns`는 기본 노이즈가 낮은 구간에서도 색조 변화가 들리도록 게이팅과 대역 가중을 상향
  - 최신 빌드에서 `Vtr/Vtw/Vc/Nn` 전용 formant-warp 레이어를 추가해 포먼트(F1~F3) 이동량을 더 직접적으로 증폭
  - 최신 빌드에서 `Vtl`은 비균일 워프(저역 < 중고역) + 유성/구간(time/region) 게이팅으로 동작하도록 변경
  - 최신 빌드에서 `Vtl`은 `g`(Gender) 전역 이동과 직교화되어 두 플래그의 캐릭터 겹침을 줄임

## 후처리
- 적응형 라우드니스 정규화 (`RMS`, `p95`, `p99.5`).
  - `Ln`으로 정규화 개입 강도 조절.
- 텐션 연동 리미팅.
- 최종 hard peak guard (`p99.9` 기반).
- 짧은 fade in/out 적용.

## OpenUtau 배포 동작
- `resamp` 빌드 후 자동 복사:
  - `C:/Users/oyh57/SODAsoo1/VocalSynth/OpenUtauV-win-x64/Resamplers/T_Sampler.exe`
  - `C:/Users/oyh57/SODAsoo1/VocalSynth/OpenUtauV-win-x64/Resamplers/T_Sampler.yaml`
- `wavtool_probe` 빌드 후 자동 복사:
  - `C:/Users/oyh57/SODAsoo1/VocalSynth/OpenUtauV-win-x64/Resamplers/wavtool_probe.exe`
- `wavtool` 빌드 후 자동 복사:
  - `C:/Users/oyh57/SODAsoo1/VocalSynth/OpenUtauV-win-x64/Wavtools/V_wavtool.exe`
  - `C:/Users/oyh57/SODAsoo1/VocalSynth/OpenUtauV-win-x64/Resamplers/V_wavtool.exe`
## Wavtool M0 (인터페이스 캡처)
- 목적: OpenUtau 외부 wavtool 호출 인자 계약 캡처.
- 실행 파일: `wavtool_probe.exe`
- 로그: 기본 `%TEMP%/resamp_wavtool_calls.jsonl` (`RESAMP_WAVTOOL_LOG`로 변경 가능)
- 참고 문서: [WAVTOOL_M0.md](/C:/Users/oyh57/SODAsoo1/Devs/Resamp/WAVTOOL_M0.md)

## Resamp wavtool
- 목적: resampler 음색 개입 없이 음소 경계 연결 품질 개선.
- 실행 파일: `V_wavtool.exe`
- 참고 문서: [WAVTOOL.md](/C:/Users/oyh57/SODAsoo1/Devs/Resamp/WAVTOOL.md)
- 기본 모드: `WT_MODE=natural`. OpenUtau 외부 wavtool 계약대로 `.whd/.dat`를 갱신하고 최종 `output.wav`는 렌더 마지막 copy 단계에서만 생성.
- phrase 진행 길이는 wavtool duration 인자로 계산.
- `WT_MODE=natural`: fast와 같은 타이밍/완료 계약을 유지하면서 overlap 내부에 envelope, RMS 레벨 매칭, 자음 보호 OLA 적용.
- `WT_MODE=fast`: envelope/DC/phase/OLA 처리 없이 타이밍 개입과 처리 비용 최소화.
- `WT_LEVEL=1`, `WT_CV=1`: fast 경로에서 각각 레벨 매칭/자음 보호만 개별 활성화.
- `WT_ENV=1`: 고속 경로에서 envelope 적용. 기본값은 렌더 속도 우선으로 off.
- `WT_MODE=smart`/`xfade`: 경계 품질 개선 실험 모드. 발음 지연/렌더 속도 문제가 있으면 기본값 사용.
- `WT_PHASE=1`: 입력 skip 기반 phase 보정 실험 옵션. 발음 지연 가능성이 있어 기본값은 off.
- 롤백:
  - `WT_MODE=append`: 단순 누적
  - `WT_MODE=xfade`: 위상 보정 없는 짧은 OLA
  - `WT_STRICT=1`: 오류 시 fallback 없이 실패 반환
- 디버그:
  - `WT_LOG=1` 또는 `WT_DEBUG=1`
  - 기본 로그: `%TEMP%/V_wavtool_calls.jsonl`
  - 로그 경로 변경: `WT_DEBUG_LOG=<path>`

## 운영 참고
- 밝기 `B` alias는 제거되었고 `Bi`만 사용.
- `modulation` 인수는 인위적 LFO 비브라토 합성에 사용하지 않음.
- 커브 베이크/슬롯 플래그(`--bake`, `Tna..Tnd`, `Bha..Bhd`, `Pca..Pcd`)는 폐기되어 현재 미지원.
- 외부 Classic resampler 경로는 호출 1회당 플래그 1세트만 전달됨(음소 내부 연속 커브 자동화 없음).
- `Mo` 적용 확인은 stderr의 다음 두 줄로 판별:
  - `flags parsed: ... Mo=...`
  - `WORLD synthesized: ... Mo=... mo_eff=... mo_ratio=... mo_blend=... Vtl=... Vtr=... Vtw=... Vc=... Nn=... vtr_eff=... vtw_eff=... vc_amt=... nn_amt=... Ns=... ns_eff=...`
- OpenUtau `expression_filter: true`에서는 manifest key로 지원 플래그를 필터링하므로, `T_Sampler.yaml`의 key/abbr 불일치 시 플래그가 조용히 누락될 수 있음.
  - 현재 key/abbr를 `bi/vcs/nn/cstb` 포함 동일 값으로 정렬해 `Vc/Nn` 누락 문제를 방지.

## ML 확장 설계
- GMM 기반 음색 변조 설계 문서: [GMM_DESIGN.md](/C:/Users/oyh57/SODAsoo1/Devs/Resamp/GMM_DESIGN.md)
- 방향: HSMM 대체가 아니라, 기존 규칙기반 위에 GMM delta 보정 레이어를 추가하는 하이브리드 접근.
