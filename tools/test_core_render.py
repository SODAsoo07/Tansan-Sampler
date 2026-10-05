"""Native render/cache regression and baseline/candidate listening files (stdlib only)."""
import argparse
from array import array
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
import wave


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pcm(path):
    with wave.open(str(path), 'rb') as f:
        assert f.getsampwidth() == 2
        rate, frames = f.getframerate(), f.getnframes()
        data = array('h', f.readframes(frames))
    return rate, data


def write_pcm(path, rate, data):
    with wave.open(str(path), 'wb') as f:
        f.setparams((1, 2, rate, 0, 'NONE', 'not compressed'))
        f.writeframes(array('h', data).tobytes())


def choose_oto(root, alias=None):
    path = root / 'oto.ini'
    raw = path.read_bytes()
    for encoding in ('utf-8-sig', 'cp932', 'cp949'):
        try:
            text = raw.decode(encoding)
        except UnicodeError:
            continue
        for line_number, line in enumerate(text.splitlines(), 1):
            if '=' not in line:
                continue
            name, rhs = line.split('=', 1)
            fields = rhs.split(',')
            if alias is not None and fields[0] != alias:
                continue
            source = root / name
            if len(fields) < 6 or not source.is_file():
                continue
            try:
                offset, consonant, cutoff, preutter, overlap = map(float, fields[1:6])
                with wave.open(str(source), 'rb') as f:
                    duration = f.getnframes() * 1000 / f.getframerate()
                end = offset - cutoff if cutoff < 0 else duration - cutoff
                if offset < 0 or consonant < 5 or consonant > 350 or end <= offset + consonant + 50:
                    continue
            except (ValueError, wave.Error, OSError):
                continue
            return dict(source=source, oto=path, line=line_number, alias=fields[0],
                        offset=offset, consonant=consonant, cutoff=cutoff,
                        preutter=preutter, overlap=overlap, original_line=line,
                        source_sha256=sha(source), oto_sha256=sha(path))
    raise RuntimeError(f'No suitable readable PCM oto entry in {root}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--resampler', required=True, type=Path)
    ap.add_argument('--baseline', required=True, type=Path)
    ap.add_argument('--wavtool', required=True, type=Path)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--bank', action='append', type=Path, default=[])
    ap.add_argument('--alias', action='append', default=[], help='Exact alias for each --bank, in the same order.')
    args = ap.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    sampler, baseline, wavtool = args.resampler.resolve(), args.baseline.resolve(), args.wavtool.resolve()
    source = output / '테스트_source.wav'
    rate = 16000
    write_pcm(source, rate, [int(7000 * math.sin(2 * math.pi * 220 * i / rate)) for i in range(rate)])
    rows, checks = [], []
    source_hash = sha(source)
    base_env = {**os.environ, 'RESAMP_ANALYSIS_CACHE': '0', 'RESAMP_VERBOSE': '1',
                'RESAMP_DIAGNOSTICS': '1'}

    def render(name, binary=sampler, input_wav=source, flags='', offset=100,
               length=300, consonant=50, cutoff=-400, velocity=100, bend='AA', tempo='!120', env=None):
        dest = output / (name + '.wav')
        log = output / (name + '.log')
        argv = [str(binary), str(input_wav), str(dest), 'A4', str(velocity), flags or '_',
                str(offset), str(length), str(consonant), str(cutoff), '100', '0', tempo, bend]
        start = time.perf_counter()
        proc = subprocess.run(argv, env={**base_env, 'RESAMP_DEBUG_LOG': str(log), **(env or {})},
                              capture_output=True, timeout=60,
                              creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        elapsed = time.perf_counter() - start
        stderr = proc.stderr.decode('utf-8', errors='replace')
        (output / (name + '.stderr.txt')).write_text(stderr, encoding='utf-8')
        log_text = log.read_text(encoding='utf-8') if log.exists() else ''
        row = dict(name=name, argv=argv, returncode=proc.returncode, elapsed_seconds=elapsed,
                   fallback='[FALLBACK_SILENCE]' in log_text, path=str(dest),
                   environment_overrides=env or {})
        if dest.exists():
            sr, data = pcm(dest)
            row.update(rate=sr, frames=len(data), peak=max(map(abs, data), default=0) / 32768,
                       rms=math.sqrt(sum(x*x for x in data) / max(1, len(data))) / 32768,
                       sha256=sha(dest))
        rows.append(row)
        return row, stderr, log_text

    def check(name, ok):
        checks.append(dict(name=name, passed=bool(ok)))
        if not ok:
            print('FAIL', name, flush=True)

    a, _, _ = render('cutoff_negative', cutoff=-400)
    b, _, _ = render('cutoff_positive', cutoff=500)
    check('equivalent_cutoffs_equal_audio', a['sha256'] == b['sha256'])
    for velocity in (0, 50, 100, 150, 200):
        row, text, _ = render(f'velocity_{velocity}', velocity=velocity, length=150, consonant=100)
        timing = re.search(r'consonant_tgt_ms=([\d.e+-]+)', text)
        expected = 100 * 2 ** (1 - velocity / 100)
        check(f'velocity_{velocity}_fixed_timing', timing and abs(float(timing[1]) - expected) < .001)
        check(f'velocity_{velocity}_native_wav', row['returncode'] == 0 and not row['fallback'] and row['peak'] > .01)
    for name, offset, cutoff, input_wav in [('eof', 1000, 0, source), ('beyond_eof', 1100, 0, source),
                                           ('huge', 1e100, -300, source), ('negative', -100, -300, source),
                                           ('bad_cutoff', 100, 1000, source)]:
        row, _, _ = render(name, offset=offset, cutoff=cutoff, input_wav=input_wav)
        check(name + '_silence', row['returncode'] == 0 and row['fallback'] and row['frames'] == 4800 and row['peak'] == 0)
    empty = output / 'empty.wav'
    write_pcm(empty, rate, [])
    row, _, _ = render('empty_source', input_wav=empty)
    check('empty_source_silence', row['returncode'] == 0 and row['fallback'] and row['peak'] == 0)
    for tempo in ('!0', '!-120', '!nan', '!inf'):
        row, _, _ = render('tempo_' + tempo[1:], tempo=tempo)
        check(tempo + '_rejected', row['returncode'] != 0)

    # Same-key process writes, existing temp protection, unusable cache fallback.
    cache = output / 'cache'
    cache.mkdir()
    active_tmp = cache / 'other_process.tmp'
    active_tmp.write_bytes(b'active writer marker')
    cache_env = {'RESAMP_ANALYSIS_CACHE': '1', 'RESAMP_CACHE_DIR': str(cache)}
    with ThreadPoolExecutor(max_workers=4) as pool:
        concurrent = list(pool.map(lambda i: render(f'concurrent_{i}', env=cache_env), range(4)))
    check('cache_concurrent_writers', all(r[0]['returncode'] == 0 and not r[0]['fallback'] for r in concurrent))
    check('active_tmp_preserved', active_tmp.read_bytes() == b'active writer marker')
    check('owned_tmp_cleaned', list(cache.glob('*.tmp')) == [active_tmp])
    row, text, _ = render('cache_hit', env=cache_env)
    check('cache_readback', 'WORLD cache hit:' in text and row['returncode'] == 0)
    blocked = output / 'not_a_directory'
    blocked.write_bytes(b'cache unavailable')
    row, _, _ = render('cache_unavailable', env={'RESAMP_ANALYSIS_CACHE': '1', 'RESAMP_CACHE_DIR': str(blocked)})
    check('cache_failure_still_renders', row['returncode'] == 0 and not row['fallback'] and row['peak'] > .01)
    row, text, _ = render('rs100', flags='Rs100')
    stats = re.search(r'\[RS_AP\] region=2 bins=(\d+) before=([\d.e+-]+) after=([\d.e+-]+)', text)
    check('rs_vowel_ap_reduced', stats and int(stats[1]) > 0 and float(stats[3]) < float(stats[2]))

    joined_unicode = output / '연결_テスト.wav'
    proc = subprocess.run([str(wavtool), str(joined_unicode), str(source), '0', '240@120+0',
        '0', '5', '35', '0', '100', '100', '0', '50', '0', '100', '100'],
        env={**os.environ, 'WT_STRICT': '1'}, capture_output=True, timeout=30,
        creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    whd, dat = Path(str(joined_unicode) + '.whd'), Path(str(joined_unicode) + '.dat')
    check('wavtool_unicode_input_and_output', proc.returncode == 0 and whd.exists() and dat.exists())

    banks = []
    for index, root in enumerate(args.bank):
        case = choose_oto(root.resolve(), args.alias[index] if index < len(args.alias) else None)
        banks.append(case)
        print(f'Rendering bank {index + 1}: {root.name}', flush=True)
        for length in (150, 500, 1200):
            for version, binary in [('baseline', baseline), ('candidate', sampler)]:
                row, _, _ = render(f'bank{index}_{length}_{version}', binary=binary,
                    input_wav=case['source'], offset=case['offset'], consonant=case['consonant'],
                    cutoff=case['cutoff'], length=length)
                check(row['name'] + '_renders', row['returncode'] == 0 and not row['fallback'] and row['peak'] > .001)
                # Whole-file RMS matched copies for listening; raw WAVs stay intact.
                sr, data = pcm(Path(row['path']))
                gain = min(.10 / max(row['rms'], 1e-9), .95 / max(row['peak'], 1e-9))
                write_pcm(output / (row['name'] + '_matched.wav'), sr,
                          [max(-32768, min(32767, round(x * gain))) for x in data])
        for version in ('baseline', 'candidate'):
            joined = output / f'bank{index}_{version}_joined.wav'
            # Controlled three-note envelope/overlap fixture, not an actual user score.
            for repeat in range(3):
                proc = subprocess.run([str(wavtool), str(joined),
                    str(output / f'bank{index}_500_{version}.wav'), '0', '240@120+0',
                    '0', '5', '35', '0', '100', '100', '0', '50', '0', '100', '100'],
                    capture_output=True, timeout=30,
                    creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                check(f'bank{index}_{version}_join{repeat}', proc.returncode == 0)
            whd, dat = Path(str(joined) + '.whd'), Path(str(joined) + '.dat')
            if whd.exists() and dat.exists():
                joined.write_bytes(whd.read_bytes() + dat.read_bytes())
            sr, data = pcm(joined)
            check(f'bank{index}_{version}_joined_audio', len(data) > sr / 2 and max(map(abs, data)) > 0)
        check(f'bank{index}_source_preserved', sha(case['source']) == case['source_sha256'] and
              sha(case['oto']) == case['oto_sha256'])
        if index == 1:
            variants = [('no_normalization', '', {'RESAMP_NORMALIZATION': '0'}),
                        ('no_boundary_guard', '', {'RESAMP_BOUNDARY_GUARD': '0'}),
                        ('legacy_boundary_boost', '', {'RESAMP_BOUNDARY_BOOST': '1'}),
                        ('no_boundary_attenuation', '', {'RESAMP_BOUNDARY_ATTENUATION': '0'}),
                        ('legacy_loop', '', {'RESAMP_STABLE_LOOP': '0'}),
                        ('no_vowel_anchor', '', {'RESAMP_VOWEL_ANCHOR': '0'}),
                        ('no_temporal_smoothing', '', {'RESAMP_TEMPORAL_SMOOTHING': '0'}),
                        ('no_loop_seam', '', {'RESAMP_LOOP_SEAM': '0'}),
                        ('legacy_transition', '', {'RESAMP_EXPAND_TRANSITION': '1'}),
                        ('legacy_fixed_compression', '', {'RESAMP_COMPRESS_FIXED': '1'}),
                        ('rs100', 'Rs100', {}),
                        ('rs100_no_restore', 'Rs100', {'RESAMP_RS_AP_RESTORE': '0'}),
                        ('rs100_no_floor', 'Rs100', {'RESAMP_RS_AP_FLOOR': '0'})]
            for variant, flags, env in variants:
                row, _, _ = render('ablation_' + variant, input_wav=case['source'],
                    offset=case['offset'], consonant=case['consonant'], cutoff=case['cutoff'],
                    length=500, flags=flags, env=env)
                check(row['name'] + '_renders', row['returncode'] == 0 and not row['fallback'] and row['peak'] > .001)
                sr, data = pcm(Path(row['path']))
                gain = min(.10 / max(row['rms'], 1e-9), .95 / max(row['peak'], 1e-9))
                write_pcm(output / (row['name'] + '_matched.wav'), sr,
                          [max(-32768, min(32767, round(x * gain))) for x in data])
    check('synthetic_source_preserved', sha(source) == source_hash)
    report = dict(passed=all(c['passed'] for c in checks), checks=checks, renders=rows,
                  banks=banks, resampler_sha256=sha(sampler), baseline_sha256=sha(baseline),
                  wavtool_sha256=sha(wavtool), listening_verified=False)
    (output / 'results.json').write_text(json.dumps(report, default=str, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'], checks=len(checks), renders=len(rows),
                         failures=[c['name'] for c in checks if not c['passed']]), ensure_ascii=False), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
