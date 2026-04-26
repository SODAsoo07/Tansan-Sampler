# Wavtool M0 (호출 인자 캡처) 가이드

## 목적
- OpenUtau 외부 `wavtool` 호출 계약(실제 argv 순서/값)을 정확히 수집한다.
- 이후 M1~M3 구현 전에, 환경별 인자 차이로 인한 재작업을 줄인다.

## 제공 파일
- 실행 파일: `wavtool_probe.exe` (빌드 타깃: `wavtool_probe`)
- 재현 스크립트: [tools/replay_wavtool_log.ps1](/C:/Users/oyh57/SODAsoo1/Devs/Resamp/tools/replay_wavtool_log.ps1)

## 현재 동작 (M0)
- 호출 인자를 JSONL로 기록
- 최소 동작으로 출력 wav를 생성/누적(정밀 접합 알고리즘 없음)
- 즉, M0는 **연결 품질 개선용이 아니라 인터페이스 캡처용**

## 로그 파일 위치
- 기본: `%TEMP%\\resamp_wavtool_calls.jsonl`
- 변경: 환경변수 `RESAMP_WAVTOOL_LOG` 지정

예시:
```powershell
$env:RESAMP_WAVTOOL_LOG = "C:\Users\oyh57\SODAsoo1\Devs\Resamp\logs\wavtool_calls.jsonl"
```

## OpenUtau 설정 (캡처용)
1. 외부 wavtool 경로를 `...\\Resamplers\\wavtool_probe.exe`로 지정
2. 짧은 테스트 프로젝트 1회 렌더
3. JSONL 로그 확인

## 로그 포맷
로그는 1줄 1 JSON 레코드.

- invoke 레코드
  - `time`, `event=invoke`, `cwd`, `argc`, `args[]`
- result 레코드
  - `time`, `event=result`, `status`, `note`, `output`, `input`, `out_sr`, `in_sr`, `out_samples`, `in_samples`

## 로그 재현
```powershell
powershell -ExecutionPolicy Bypass -File tools\replay_wavtool_log.ps1 `
  -LogPath "C:\Users\oyh57\SODAsoo1\Devs\Resamp\logs\wavtool_calls.jsonl" `
  -WavtoolPath "C:\Users\oyh57\SODAsoo1\Devs\Resamp\build\Release\wavtool_probe.exe" `
  -Max 20
```

## 제한 사항 (중요)
- M0는 실제 UTAU 품질용 wavtool이 아니다.
- 오버랩/위상 정렬/유무성 분기/적응형 크로스페이드는 아직 없음.
- 음질 비교는 M1 이상부터 수행해야 유의미하다.

## 다음 단계
- M1: 기본 crossfade 기반 접합기
- M2: 유성 위상 정렬 + 무성 전용 분기
- M3: 경계 적응형 윈도우 + anti-click/anti-ringing
