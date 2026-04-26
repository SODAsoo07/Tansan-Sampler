# Resamp Knowledge Summary (for CQ MCP)

## Project identity
- Name: `Resamp`
- Purpose: WORLD-based resampler for UTAU/OpenUtau.
- Core pipeline: CLI parse -> source trim -> WORLD analyze -> target F0 mapping -> WORLD render -> post volume/fade -> WAV write.

## Core DSP architecture
- Analysis: DIO + StoneMask + CheapTrick + D4C.
- Analysis frame period: 2.5ms.
- CheapTrick smoothing: `q1 = -0.30`.
- Render frame period: adaptive (`0.50ms` for modulated notes, `0.65ms` normal, `0.80ms` flat).
- Connection handling:
  - consonant/transition one-pass handling
  - stable vowel loop region
  - loop boundary seam crossfade
  - additional join smoothing in consonant/transition area.

## Pitch handling
- `pitch_bend` is decoded from arg[13] (12-bit + RLE or legacy int8 base64).
- Per-sample cents contour is smoothed (zero-phase FIR) then applied to target F0.
- New global cents flag:
  - `t` = pitch cents offset (`-1200..1200`).

## Flag model (current)
- Tone/voice:
  - `g` gender/formant warp
  - `Bi` brightness
  - `Hu` husky tone
  - `Mo` mouth open
  - `Tn` tension (renamed from old `t`)
  - `Gr` growl
  - `c` voice color
- Harmonic/noise:
  - `H/Hr` harmonics (neutral around 70)
  - `N` noise level
  - `Bh` breathiness
  - `Ns` noise color
- Transition/articulation:
  - `Tr` transition length
  - `Cs` consonant stability
  - `At` attack
  - `Rl` release air
- Dynamics:
  - `P` peak compression
- Tract simulator layer:
  - `Vtl` tract length
  - `Vtr` tract resonance shift
  - `Vtw` tract focus/width
  - `Vc` constriction
  - `Nn` nasal coupling

## Tract simulator layer behavior
- Implemented inside WORLD spectral/AP shaping stage.
- Combines `Vtl/Vtr/Vtw/Vc/Nn` and `Mo` with:
  - additional tract warp (`Vtl`)
  - multi-formant center/focus shifts (`Vtr/Vtw`)
  - constriction emphasis (`Vc`)
  - nasal formant + notch behavior (`Nn`)
  - frame energy normalization to reduce loudness drift.

## Post processing
- Adaptive loudness normalization (`RMS`, `p95`, `p99.5` gates).
- Tension-aware limiting behavior.
- Final hard peak guard (`p99.9`-based).
- Very short fade in/out applied at the end.

## OpenUtau deployment behavior
- Build target `resamp` post-build auto-deploys to:
  - `C:/Users/oyh57/SODAsoo1/VocalSynth/OpenUtauV-win-x64/Resamplers/resamp.exe`
  - `C:/Users/oyh57/SODAsoo1/VocalSynth/OpenUtauV-win-x64/Resamplers/resamp.yaml`

## Operational notes
- `B` alias is removed; brightness uses `Bi` only.
- `modulation` argument is intentionally not used to synthesize artificial LFO vibrato.
- External Classic resampler path still receives one flag set per phoneme render call (no intra-phoneme curve automation).
