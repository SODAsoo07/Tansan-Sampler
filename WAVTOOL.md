# Resamp wavtool

`V_wavtool.exe`는 이 저장소의 `resamp.exe`가 만든 음소 wav 조각을 OpenUtau Classic 렌더 단계에서 누적 합성하는 전용 wavtool입니다.

## 설치 위치

Release 빌드 후 자동 복사:

- `build/Release/V_wavtool.exe`
- `C:/Users/oyh57/SODAsoo1/VocalSynth/OpenUtauV-win-x64/Wavtools/V_wavtool.exe`
- `C:/Users/oyh57/SODAsoo1/VocalSynth/OpenUtauV-win-x64/Resamplers/V_wavtool.exe`

OpenUtau에서는 Classic renderer 설정에서 wavtool을 `V_wavtool.exe`로 선택합니다.

## 지원 호출 형태

OpenUtau 외부 wavtool 호출을 기준으로 다음 형태를 처리합니다.

```text
V_wavtool.exe <output.wav> <input.wav> <skip_ms|offset_ms> <duration_ms|ticks@tempo+correction> [envelope...]
```

현재 로컬 M0 로그에서는 최소 호출 형태만 확인되었습니다.

```text
wavtool_probe.exe output.wav input.wav 0 0 100
```

따라서 `V_wavtool.exe`는 인자가 부족하거나 예상과 달라도 실패보다는 기본 append fallback으로 넘어가도록 작성했습니다. 실제 OpenUtau 프로젝트 렌더 로그가 더 쌓이면 파서와 배치 규칙을 더 좁힐 수 있습니다.

## 동작 모드

환경변수로 안전하게 롤백할 수 있습니다.

| 변수 | 값 | 동작 |
|---|---|---|
| `WT_MODE` | unset / `fast` | 기본값. 최종 output wav에 raw PCM을 overlap 위치로 직접 배치 |
| `WT_MODE` | `append` / `legacy` / `off` | `fast`와 동일한 고속 배치 |
| `WT_MODE` | `xfade` / `basic` | 실험용. 위상 정렬 없이 짧은 OLA 적용 |
| `WT_MODE` | `smart` | 실험용. adaptive overlap과 경계 분석 적용 |
| `WT_ENV` | `1` / `true` | 기본 off. 고속 경로에서 envelope gain 적용 |
| `WT_PHASE` | `1` / `true` | 실험용. 입력 skip 기반 phase 보정 사용 |
| `WT_STRICT` | `1` / `true` | 오류 시 fallback하지 않고 실패 반환 |
| `WT_LOG` / `WT_DEBUG` | `1` / `true` | 기본 off. 호출 JSONL 로그 기록 |
| `WT_DEBUG_LOG` | path | 호출 JSONL 로그 경로 지정 |

## 구현된 품질 처리

- 기본 모드는 출력 wav 전체를 재작성하지 않고 최종 output wav를 직접 갱신
- 현재 조각은 `output_length - overlap` 위치에 배치하고, phrase 진행 길이는 `duration`으로 계산
- 기본 모드는 envelope/DC/phase/OLA 처리를 하지 않아 타이밍 개입을 최소화
- `WT_MODE=smart`에서만 DC offset, envelope 근사, adaptive overlap, tail 억제 등 실험 기능 적용
- 실패 시 기본 append fallback

## 빠른 검증

```powershell
cmake --build build --config Release --target wavtool
powershell -ExecutionPolicy Bypass -File tools\test_wavtool.ps1
```

성공 시 `build/wavtool_test.wav`가 생성되고 WAV 헤더/샘플 수를 확인합니다.

## 호출 로그

`V_wavtool.exe`는 `WT_LOG=1` 또는 `WT_DEBUG=1`일 때 JSONL 로그를 남깁니다.

- 기본 위치: `%TEMP%/V_wavtool_calls.jsonl`
- 변경: `WT_DEBUG_LOG=<path>`

## 현실적인 제한

- 현재 구현은 외부 wavtool 단계에서 가능한 접합 품질 개선에 집중합니다. 보이스뱅크 oto 자동 보정이나 resampler 음색 보정은 하지 않습니다.
- OpenUtau의 외부 wavtool argv 계약은 버전과 렌더 경로에 따라 달라질 수 있어, 실제 프로젝트 렌더 로그 기반 보정이 필요합니다.
- 완전한 내장 `convergence` 동일 동작은 아닙니다. 기본값은 품질 개선보다 정박/속도 안정성을 우선합니다.
- `WT_MODE=smart`/`xfade`는 아직 실험용입니다. 발음이 늦거나 렌더가 느리면 기본값으로 되돌리는 것이 맞습니다.
- `WT_PHASE=1`은 아직 실험용입니다. 발음이 늦거나 흐려지면 끄는 것이 맞습니다.
