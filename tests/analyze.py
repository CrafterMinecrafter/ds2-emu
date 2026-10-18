#!/usr/bin/env python3
"""Offline checks for DS2Model via the `render` binary.

  python3 analyze.py <render-binary> <out-dir>

- small-signal frequency response (Turbo I / II, tone sweep)
- harmonic content and output level for sines
- aliasing estimate at 4x oversampling
- clicks on TURBO switching and DIST sweeps
- a demo WAV (synthetic plucked guitar: clean DI -> Turbo I -> Turbo II)
"""
import os
import subprocess
import sys
import tempfile

import numpy as np
from scipy import signal
from scipy.io import wavfile

FS = 44100
OS = 4
RENDER = sys.argv[1]
OUT = sys.argv[2]
os.makedirs(OUT, exist_ok=True)
TMP = tempfile.mkdtemp()


def run(x, dist, tone, level, turbo, trim=0.0, flags=()):
    """x at FS (volts-ish, 1.0 = 1 V) -> y at FS, through 4x oversampling."""
    up = signal.resample_poly(x, OS, 1, window=('kaiser', 12.0)).astype(np.float32)
    fi, fo = os.path.join(TMP, "i.f32"), os.path.join(TMP, "o.f32")
    up.tofile(fi)
    r = subprocess.run([RENDER, fi, fo, str(FS * OS), str(dist), str(tone), str(level),
                        str(turbo), str(trim), *flags], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr)
    y = np.fromfile(fo, dtype=np.float32).astype(np.float64)
    return signal.resample_poly(y, 1, OS, window=('kaiser', 12.0)), r.stderr.strip()


def response(dist, tone, turbo, flags=()):
    rng = np.random.default_rng(1)
    x = rng.standard_normal(FS * 8) * 1e-5
    y, _ = run(x, dist, tone, 1.0, turbo, flags=flags)
    x, y = x[FS:], y[FS:]
    f, pxy = signal.csd(x, y, fs=FS, nperseg=8192)
    _, pxx = signal.welch(x, fs=FS, nperseg=8192)
    return f, 20 * np.log10(np.abs(pxy / pxx) + 1e-30)


def at(f, h, hz):
    return h[np.argmin(np.abs(f - hz))]


report = []
p = report.append

# ---------------------------------------------------------------- linear response
p("== Small-signal response (DIST min, LEVEL max), dB re 1 kHz of Turbo I/tone noon ==")
f, hI = response(0.0, 0.5, 1)
_, hII = response(0.0, 0.5, 2)
_, hIflat = response(0.0, 0.5, 1, flags=("noNotch",))
_, hT0 = response(0.0, 0.0, 1)
_, hT1 = response(0.0, 1.0, 1)
ref = at(f, hI, 1000)
for hz in (50, 100, 200, 500, 1000, 2000, 4000, 8000, 12000):
    p(f"  {hz:>6} Hz  I:{at(f, hI, hz) - ref:+6.1f}  II:{at(f, hII, hz) - ref:+6.1f}"
      f"  I(no notch):{at(f, hIflat, hz) - ref:+6.1f}  toneMin:{at(f, hT0, hz) - ref:+6.1f}"
      f"  toneMax:{at(f, hT1, hz) - ref:+6.1f}")
band = (f > 200) & (f < 5000)
pk = np.argmax(np.where(band, hII, -1e9))
p(f"  Turbo II peak: {f[pk]:.0f} Hz, {hII[pk] - ref:+.1f} dB;  Turbo I notch depth at 1 kHz: {at(f, hI, 1000) - at(f, hIflat, 1000):+.1f} dB")

_, hD5 = response(0.5, 0.5, 1)
_, hD10 = response(1.0, 0.5, 1)
p(f"  gain at 1 kHz vs DIST min: noon {at(f, hD5, 1000) - ref:+.1f} dB, max {at(f, hD10, 1000) - ref:+.1f} dB")

# ---------------------------------------------------------------- sines
p("\n== 110 Hz sine, 0.2 V peak, LEVEL max ==")
t = np.arange(int(FS * 2.0)) / FS
sine = 0.2 * np.sin(2 * np.pi * 110 * t)
for turbo in (1, 2):
    for dist in (0.0, 0.5, 1.0):
        y, info = run(sine, dist, 0.5, 1.0, turbo)
        seg = y[FS:]
        spec = np.abs(np.fft.rfft(seg * np.hanning(len(seg))))
        fr = np.fft.rfftfreq(len(seg), 1 / FS)
        harm = [spec[np.argmin(np.abs(fr - 110 * k))] for k in range(1, 11)]
        thd = np.sqrt(np.sum(np.square(harm[1:]))) / harm[0]
        even = np.sqrt(harm[1] ** 2 + harm[3] ** 2) / harm[0]
        p(f"  Turbo {'I' * turbo:<2} dist={dist:.1f}: peak={np.max(np.abs(seg)):.3f} "
          f"rms={np.sqrt(np.mean(seg ** 2)):.3f} THD={100 * thd:5.1f}% even(H2,H4)={100 * even:4.1f}%  [{info.splitlines()[0]}]")

# ---------------------------------------------------------------- aliasing
p("\n== Aliasing: 3520 Hz sine, 0.3 V, DIST max (energy off the harmonic grid) ==")
x = 0.3 * np.sin(2 * np.pi * 3520 * np.arange(FS * 2) / FS)
for turbo in (1, 2):
    y, _ = run(x, 1.0, 0.5, 1.0, turbo)
    seg = y[FS // 2:]
    w = signal.windows.blackmanharris(len(seg))
    spec = np.abs(np.fft.rfft(seg * w)) ** 2
    fr = np.fft.rfftfreq(len(seg), 1 / FS)
    mask = np.ones_like(spec, bool)
    for k in range(1, 8):
        mask &= np.abs(fr - 3520 * k) > 30
    mask &= fr > 30
    ratio = 10 * np.log10(np.sum(spec[mask]) / np.sum(spec[~mask]))
    worst = 10 * np.log10(np.max(spec[mask]) / np.max(spec))
    p(f"  Turbo {'I' * turbo:<2}: alias energy {ratio:+.1f} dB re harmonics, worst alias line {worst:+.1f} dB")

# ---------------------------------------------------------------- pluck synth
def pluck(freq, dur, amp=0.25, seed=0):
    """Karplus-Strong string, roughly guitar-DI-like."""
    n = int(FS * dur)
    d = max(2, int(round(FS / freq - 0.5)))
    rng = np.random.default_rng(seed)
    buf = rng.uniform(-1, 1, d)
    buf = signal.lfilter([0.6, 0.4], [1], buf)
    buf -= buf.mean()
    buf = list(buf)
    out = np.empty(n)
    for i in range(n):
        j = i % d
        out[i] = buf[j]
        buf[j] = 0.997 * 0.5 * (buf[j] + buf[(i + 1) % d])
    env = np.minimum(1.0, np.exp(-(np.arange(n) / FS - (dur - 0.08)) / 0.03))  # release
    return amp * out * env


def riff():
    notes = [  # (start beat, dur beats, [freqs])  power chords + a short lead
        (0, 1.5, [82.41, 123.47, 164.81]), (1.5, 0.5, [82.41, 123.47]),
        (2, 1, [98.0, 146.83, 196.0]), (3, 1, [110.0, 164.81, 220.0]),
        (4, 1.5, [82.41, 123.47, 164.81]), (5.5, 0.5, [82.41, 123.47]),
        (6, 2, [73.42, 110.0, 146.83]),
        (8, 0.5, [329.63]), (8.5, 0.5, [392.0]), (9, 0.5, [440.0]), (9.5, 1.5, [493.88]),
        (11, 0.5, [440.0]), (11.5, 1.5, [392.0]),
    ]
    bpm = 132
    beat = 60 / bpm
    total = int(FS * 14 * beat)
    out = np.zeros(total)
    for k, (s, dur, fs_) in enumerate(notes):
        i0 = int(s * beat * FS)
        for j, fr in enumerate(fs_):
            x = pluck(fr, dur * beat + 0.4, amp=0.18 if len(fs_) > 1 else 0.3, seed=k * 10 + j)
            x = x[: total - i0 - int(j * 0.006 * FS)]
            st = i0 + int(j * 0.006 * FS)
            out[st: st + len(x)] += x
    return out


p("\n== Clicks: TURBO toggled every 0.25 s on a sustained note; DIST swept 0->1->0 ==")
held = pluck(110.0, 3.0, amp=0.3, seed=5)
y_ref, _ = run(held, 0.6, 0.5, 0.6, 1)
y_tog, _ = run(held, 0.6, 0.5, 0.6, 1, flags=("toggle=0.25",))
y_sw, info = run(held, 0.5, 0.5, 0.6, 1, flags=("sweepDist",))
def max_jump(v):
    return np.max(np.abs(np.diff(v)))
p(f"  max sample-to-sample step: steady {max_jump(y_ref):.3f}, toggling {max_jump(y_tog):.3f}, dist sweep {max_jump(y_sw):.3f}")

# ---------------------------------------------------------------- demo wav
di = riff()
gap = np.zeros(int(0.6 * FS))
y1, _ = run(di, 0.6, 0.55, 0.45, 1)
y2, _ = run(di, 0.6, 0.55, 0.45, 2)
demo = np.concatenate([di, gap, y1, gap, y2])
pk = np.max(np.abs(demo))
demo = demo / pk * 0.89
wavfile.write(os.path.join(OUT, "demo_clean_turboI_turboII.wav"), FS, (demo * 32767).astype(np.int16))
p(f"\n== Demo: clean DI peak {np.max(np.abs(di)):.2f} V; Turbo I rms {np.sqrt(np.mean(y1**2)):.3f}, "
  f"Turbo II rms {np.sqrt(np.mean(y2**2)):.3f}; normalised by {pk:.3f}")

# ---------------------------------------------------------------- plot
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
for h, lab in ((hI, "Turbo I"), (hII, "Turbo II"), (hT0, "Turbo I, tone min"), (hT1, "Turbo I, tone max")):
    ax[0].semilogx(f[1:], h[1:] - ref, label=lab)
ax[0].set_xlim(20, 20000); ax[0].set_ylim(-30, 20); ax[0].grid(True, which="both", alpha=.3)
ax[0].set_title("Small-signal response, DIST min"); ax[0].set_xlabel("Hz"); ax[0].set_ylabel("dB"); ax[0].legend()
seg = slice(FS + 2000, FS + 2000 + int(FS * 0.03))
for turbo in (1, 2):
    y, _ = run(sine, 1.0, 0.5, 1.0, turbo)
    ax[1].plot(np.arange(seg.stop - seg.start) / FS * 1000, y[seg], label=f"Turbo {'I' * turbo}")
ax[1].plot(np.arange(seg.stop - seg.start) / FS * 1000, sine[seg], "k--", lw=.8, label="input")
ax[1].set_title("110 Hz, 0.2 V, DIST max"); ax[1].set_xlabel("ms"); ax[1].legend(); ax[1].grid(alpha=.3)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "analysis.png"), dpi=110)

txt = "\n".join(report)
print(txt)
with open(os.path.join(OUT, "report.txt"), "w") as fh:
    fh.write(txt + "\n")
