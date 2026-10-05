"""Exercise native UTAU cutoff handling; uses an explicit WAV without changing it."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import wave


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resampler', type=Path, required=True)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    sampler, source, output = args.resampler.resolve(), args.input.resolve(), args.output.resolve()
    source_sha = hashlib.sha256(source.read_bytes()).hexdigest()
    with wave.open(str(source), 'rb') as wav:
        rate, frames = wav.getframerate(), wav.getnframes()
    duration = frames * 1000.0 / rate
    if duration < 1000:
        parser.error('Use a PCM WAV at least one second long.')
    output.mkdir(parents=True, exist_ok=False)
    # Expected endpoints name the UTAU meaning, independent of the sampler's implementation.
    cases = [
        ('negative_length', 100, -250, int(350 * rate / 1000), False),
        ('positive_right_blank', 100, 250, int(frames - 250 * rate / 1000), False),
        ('zero_eof', 100, 0, frames, False),
        ('negative_eof_clamp', 100, -duration, frames, False),
        ('positive_minimum_span', 100, duration + 100, int(100 * rate / 1000) + 1, False),
        ('fractional_offset_length', 100.125, -250.375, int(350.5 * rate / 1000), False),
        ('negative_flags_omitted', 100, -250, int(350 * rate / 1000), True),
        ('positive_flags_omitted', 100, 250, int(frames - 250 * rate / 1000), True),
    ]
    results = []
    for name, offset, cutoff, expected, omit_flags in cases:
        log, rendered = output / f'{name}.log', output / f'{name}.wav'
        argv = [str(sampler), str(source), str(rendered), 'C4', '100']
        if not omit_flags:
            argv.append('')
        argv += [str(offset), '150', '90', str(cutoff), '100', '0', '!120', 'AA']
        env = {**os.environ, 'RESAMP_DEBUG_LOG': str(log), 'RESAMP_VERBOSE': '0'}
        proc = subprocess.run(argv, env=env, capture_output=True, timeout=45,
                              creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        text = log.read_text(encoding='utf-8') if log.exists() else ''
        trim = re.search(r'playback_start=(\d+) playback_end=(\d+) analysis_start=(\d+) analysis_end=(\d+)', text)
        actual = int(trim.group(2)) if trim else None
        with wave.open(str(rendered), 'rb') as wav:
            output_frames = wav.getnframes()
        passed = proc.returncode == 0 and actual == expected and output_frames == round(rate * .15)
        results.append(dict(case=name, passed=passed, expected_end=expected, actual_end=actual,
                            argv=argv, returncode=proc.returncode, output_frames=output_frames))
    assert hashlib.sha256(source.read_bytes()).hexdigest() == source_sha
    report = dict(passed=all(row['passed'] for row in results), results=results,
                  input_sha256=source_sha, resampler_sha256=hashlib.sha256(sampler.read_bytes()).hexdigest())
    (output / 'results.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'passed': report['passed'], 'cases': len(results),
                      'failures': [row['case'] for row in results if not row['passed']]}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
