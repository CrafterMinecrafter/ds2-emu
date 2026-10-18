#pragma once
/*
    DS2Model.h — circuit-informed model of a two-mode "turbo" distortion
    (Boss DS-2 Turbo Distortion topology, ET board 7510952000 / VR board 7510954000).

    Header-only, C++17, no dependencies. Mono, in-place, float I/O, double inside.
    Run it at an OVERSAMPLED rate — 4x (176.4 / 192 kHz) recommended.
    Enable flush-to-zero on the audio thread (juce::ScopedNoDenormals does it).

    Signal flow (schematic designators):

      C12/R16 input buffer (Q6)
      R62 + D14/D15            1SS188 pair to ground at the booster input
      Q22 booster              CE, Rc = R65 2.2k, Re = R66 1.5k || (R67 220 + C42 10u),
                               base bias R63 100k from +4V -> collector sits ~1 V above saturation,
                               so it clips hard on one side and soft on the other
      TURBO I  (Q14 on)        Q22 collector -> C39 -> op-amp input (full range)
      TURBO II (Q15 on)        op-amp input = node "X" summing
                                 - C41 2.2n + R64 10k        (treble path)
                                 - Q23 Sallen-Key (R69/R70 22k, C43 47n, C44 1n; resonant ~1 kHz)
                                   -> C45 22n -> R68 10k
                               loaded by R71 / R49 (100k to +4V)
      Discrete op-amp          Q16/Q19 JFET pair, Q17, Q18; non-inverting
                               Zg = R54 1k + C33 0.22u           (bass cut below ~720 Hz)
                               Zf = (R55 1k + DIST VR1 250k) || C32 100p
                               TURBO I: Q20 shorts R56, putting D8/D9 + C34 2.2n into Zf (soft clip)
                               TURBO II: that branch hangs off R56 1M (effectively out)
      C35 -> R57 4.7k -> D11/D12   hard clip to ground, loaded by the tone stage
      Tone stage (Q13)         inverting, Ri = R58 15k, Rf = R46 33k || C29 470p;
                               treble branch: R59 1k - VR2 100k - R45 4.7k, wiper -> C37 6.8n -> summing node
      Bridged-T                R44/R33 22k, C28 4.7n; TURBO I: Q10 grounds C21 10n at the centre (~1 kHz cut)
      Q12 -> R42 10k -> C26 1n || (R41 10k + C25 6.8n)   fixed high-shelf cut
      LEVEL VR3 50k (A)        wiper in, MAX end out, loaded by ~50k

    Mode assignment was traced from the switch logic: SW1 "I" grounds the Q9 inverter input,
    which drives Q14, Q20 and Q10 on; Q8 inverts that for Q15.

    Values tagged [assumed] could not be read from the scan or are modelling choices.
    All of them live in DS2Model::Config so the model can be calibrated against a real pedal.
*/

#include <algorithm>
#include <cmath>

namespace ds2
{

constexpr double kPi = 3.14159265358979323846;

//==============================================================================
// Linear building blocks (analog prototype -> bilinear transform)

inline double bilinearK (double fs, double fWarp)
{
    if (fWarp <= 0.0 || fWarp >= 0.45 * fs)
        return 2.0 * fs;
    const double w = 2.0 * kPi * fWarp;
    return w / std::tan (w / (2.0 * fs));
}

// H(s) = (b0 + b1 s) / (a0 + a1 s)
struct FirstOrder
{
    double b0 = 1.0, b1 = 0.0, a1 = 0.0, z = 0.0;

    void setAnalog (double B0, double B1, double A0, double A1, double fs, double fWarp = 0.0)
    {
        const double K = bilinearK (fs, fWarp);
        const double n0 = B0 + B1 * K, n1 = B0 - B1 * K;
        const double d0 = A0 + A1 * K, d1 = A0 - A1 * K;
        b0 = n0 / d0;  b1 = n1 / d0;  a1 = d1 / d0;
    }
    void setHighpass (double rc, double fs) { setAnalog (0.0, rc, 1.0, rc, fs); }

    double process (double x)
    {
        const double y = b0 * x + z;
        z = b1 * x - a1 * y;
        return y;
    }
    void reset() { z = 0.0; }
};

// H(s) = (b0 + b1 s + b2 s^2) / (a0 + a1 s + a2 s^2)
struct Biquad
{
    double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0, z1 = 0.0, z2 = 0.0;

    void setAnalog (double B0, double B1, double B2, double A0, double A1, double A2,
                    double fs, double fWarp = 0.0)
    {
        const double K = bilinearK (fs, fWarp), K2 = K * K;
        const double n0 = B0 + B1 * K + B2 * K2, n1 = 2.0 * (B0 - B2 * K2), n2 = B0 - B1 * K + B2 * K2;
        const double d0 = A0 + A1 * K + A2 * K2, d1 = 2.0 * (A0 - A2 * K2), d2 = A0 - A1 * K + A2 * K2;
        b0 = n0 / d0;  b1 = n1 / d0;  b2 = n2 / d0;  a1 = d1 / d0;  a2 = d2 / d0;
    }

    double process (double x)
    {
        const double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
    void reset() { z1 = z2 = 0.0; }
};

// Zero-delay one-pole low-pass, safe to modulate.
struct OnePoleLP
{
    double g = 1.0, s = 0.0;
    void setCutoff (double fc, double fs)
    {
        const double wd = std::tan (kPi * std::min (fc, 0.45 * fs) / fs);
        g = wd / (1.0 + wd);
    }
    double process (double x)
    {
        const double v = (x - s) * g;
        const double y = v + s;
        s = y + v;
        return y;
    }
    void reset() { s = 0.0; }
};

//==============================================================================
// Nonlinear helpers

struct Diode
{
    double Is  = 2.52e-9;            // 1N4148-class silicon; 1SS133 is equivalent   [assumed]
    double nVt = 1.752 * 0.025852;   // emission coefficient * thermal voltage
};

#ifdef DS2_STATS
struct SolverStats { long long calls = 0, iters = 0; int maxIters = 0; };
inline SolverStats& solverStats() { static SolverStats s; return s; }
#endif

// Safeguarded Newton for an increasing f on [lo, hi] with f(lo) <= 0 <= f(hi).
template <typename F>
inline double solveMonotone (F&& f, double lo, double hi, double x, int maxIter = 50)
{
    if (! (x > lo && x < hi))
        x = 0.5 * (lo + hi);

    int it = 0;
    while (it < maxIter)
    {
        ++it;
        double fx, dfx;
        f (x, fx, dfx);
        if (fx > 0.0) hi = x; else lo = x;

        double xn = x - fx / dfx;
        if (! (xn > lo && xn < hi))
            xn = 0.5 * (lo + hi);

        const bool done = std::abs (xn - x) < 1.0e-10;
        x = xn;
        if (done)
            break;
    }
#ifdef DS2_STATS
    auto& st = solverStats();
    ++st.calls;  st.iters += it;  st.maxIters = std::max (st.maxIters, it);
#endif
    return x;
}

// Wright omega: the w with w + ln(w) = z. One Fritsch iteration from an asymptotic guess
// (~1e-6 relative); the caller polishes the diode equation itself.
inline double wrightOmega (double z)
{
    double w = (z > 1.0) ? z - std::log (z) + std::log (z) / z
                         : std::log1p (std::exp (z));
    {
        const double r   = z - w - std::log (w);
        const double wp1 = 1.0 + w;
        const double q   = 2.0 * wp1 * (wp1 + (2.0 / 3.0) * r);
        w *= 1.0 + (r / wp1) * (q - r) / (q - 2.0 * r);
    }
    return w;
}

// Solve  Is (e^(v/nVt) - e^(-v/nVt)) + G v = a   for a >= 0, G > 0  ->  v in [0, a/G].
// Closed form via Wright omega (reverse diode approximated), then Newton polish on the exact equation.
inline double solveDiodePair (double a, double G, const Diode& d)
{
    if (a <= 0.0)
        return 0.0;

    const double n  = d.nVt;
    const double gn = G * n;
    const double b  = a + d.Is;
    const double z  = std::log (d.Is / gn) + b / gn;
    double v = b / G - n * wrightOmega (z);

    const double vMax = a / G;
    int it = 0;
    while (it < 4)
    {
        ++it;
        v = std::min (std::max (v, 0.0), vMax);
        const double e  = std::exp (v / n), ie = 1.0 / e;
        const double f  = d.Is * (e - ie) + G * v - a;
        const double df = d.Is / n * (e + ie) + G;
        const double dv = f / df;
        v -= dv;
        if (std::abs (dv) < 3.0e-5)   // Newton is quadratic: what's left is ~ dv^2 / nVt < 0.1 uV
            break;
    }
#ifdef DS2_STATS
    auto& st = solverStats();
    ++st.calls;  st.iters += it;  st.maxIters = std::max (st.maxIters, it);
#endif
    v = std::min (std::max (v, 0.0), vMax);

    if (! std::isfinite (v))   // should never happen; fall back to bracketed Newton
    {
        auto fn = [&] (double x, double& fx, double& dfx)
        {
            const double e = std::exp (x / n), ie = 1.0 / e;
            fx  = d.Is * (e - ie) + G * x - a;
            dfx = d.Is / n * (e + ie) + G;
        };
        v = solveMonotone (fn, 0.0, std::min (vMax, n * std::asinh (a / (2.0 * d.Is))), 0.0);
    }
    return v;
}

// Thevenin source (vth, rth) into an antiparallel diode pair to ground:
//     (v - vth) / rth + 2 Is sinh(v / nVt) = 0
inline double solveShuntDiodePair (double vth, double rth, const Diode& d)
{
    const double v = solveDiodePair (std::abs (vth) / rth, 1.0 / rth, d);
    return vth >= 0.0 ? v : -v;
}

// Smooth limiter with a fairly hard knee: x / (1 + |x/lim|^4)^(1/4)
inline double softLimit (double x, double lim)
{
    const double r = x / lim, r2 = r * r;
    return x / std::sqrt (std::sqrt (1.0 + r2 * r2));
}

inline double asymLimit (double x, double limUp, double limDown)
{
    return x >= 0.0 ? softLimit (x, limUp) : softLimit (x, limDown);
}

//==============================================================================
// Discrete op-amp gain stage: ideal op-amp + GBW limit + asymmetric output swing.
// Capacitors use trapezoidal companion models (consistent with the bilinear filters).
class GainStage
{
public:
    double R54 = 1.0e3, C33 = 0.22e-6, C32 = 100.0e-12, C34 = 2.2e-9, R56 = 1.0e6;
    double gbw = 1.0e6, swingUp = 3.0, swingDown = 2.3;
    Diode diode;

    void prepare (double sampleRate)
    {
        fs  = sampleRate;
        g33 = 2.0 * C33 * fs;
        g32 = 2.0 * C32 * fs;
        g34 = 2.0 * C34 * fs;
        reset();
    }

    void reset()
    {
        J33 = J32 = 0.0;
        resetDiodeBranch();
        lp.reset();
    }

    void resetDiodeBranch() { J34 = 0.0; vd = 0.0; }

    // Control rate. Closed-loop bandwidth ~ GBW / noise gain.
    void setFeedbackResistance (double rf)
    {
        Rf = std::max (rf, 10.0);
        lp.setCutoff (gbw / (1.0 + Rf / R54), fs);
    }

    double process (double vin, bool diodeBranch)
    {
        // Gain leg R54 + C33 to ground, driven by V- = V+ = vin.
        const double vC33 = (vin / R54 + J33) / (1.0 / R54 + g33);
        const double ig   = (vin - vC33) / R54;
        J33 = 2.0 * g33 * vC33 - J33;

        double vf;
        if (diodeBranch)
        {
            // Zf = Rf || C32 || [(D8 || D9) in series with C34]
            const double G1  = 1.0 / Rf + g32;
            const double den = 1.0 + g34 / G1;
            const double K1  = g34 / den;
            const double K0  = (g34 * (ig + J32) / G1 - J34) / den;

            // 2 Is sinh(vd/nVt) + K1 vd = K0   (a tiny leak keeps it well-posed)
            constexpr double leak = 1.0e-9;
            const double v = solveDiodePair (std::abs (K0), K1 + leak, diode);
            vd = K0 >= 0.0 ? v : -v;

            const double i34 = K0 - K1 * vd;
            vf = (ig + J32 - i34) / G1;
            J34 = 2.0 * g34 * (vf - vd) - J34;
        }
        else
        {
            // Branch is behind R56 1M: above ~70 Hz that is just 1M across Zf.
            vf = (ig + J32) / (1.0 / Rf + 1.0 / R56 + g32);
        }

        J32 = 2.0 * g32 * vf - J32;

        return asymLimit (lp.process (vin + vf), swingUp, swingDown);
    }

private:
    double fs = 48000.0, Rf = 1.0e3;
    double g33 = 0.0, g32 = 0.0, g34 = 0.0;
    double J33 = 0.0, J32 = 0.0, J34 = 0.0, vd = 0.0;
    OnePoleLP lp;
};

//==============================================================================
// Hard clipper (R57 + D11/D12) solved together with the tone stage that loads it.
// Tone stage: ideal inverting amp (Q13), summing node T at virtual ground.
//   U --R58(Ri)--> T,   U --R59(Ra)--[pot x]--W--[pot 1-x]--R45(Rb)--> vo,
//   W --C37--> T,       vo --R46(Rf) || C29--> T
class ClipToneStage
{
public:
    double R57 = 4.7e3, Ri = 15e3, Rf = 33e3, Ra = 1e3, Rb = 4.7e3, pot = 100e3;
    double C37 = 6.8e-9, C29 = 470e-12;
    Diode diode;

    void prepare (double sampleRate)
    {
        g37 = 2.0 * C37 * sampleRate;
        g29 = 2.0 * C29 * sampleRate;
        reset();
    }
    void reset() { J37 = J29 = vU = 0.0; }

    // knob 0..1, CW (HI) = treble boost. HI end of VR2 is on the input side.
    void setTone (double knob)
    {
        const double x = 1.0 - knob;          // wiper distance from the input-side end
        G1 = 1.0 / (Ra + x * pot);
        G2 = 1.0 / ((1.0 - x) * pot + Rb);
    }

    // vsrc: op-amp output after C35. Returns the tone-stage output (inverted, like Q13).
    double process (double vsrc)
    {
        const double Gf  = 1.0 / Rf + g29;
        const double kWd = G1 + G2 + g37 + G2 * g37 / Gf;
        const double aW  = (G1 - G2 / (Ri * Gf)) / kWd;
        const double bW  = (G2 * (J37 + J29) / Gf + J37) / kWd;

        // the tone stage seen from the clip node: i = Geq * U - Jeq
        const double Geq = 1.0 / Ri + G1 * (1.0 - aW);
        const double Jeq = G1 * bW;
        const double Gs  = 1.0 / R57 + Geq;

        vU = solveShuntDiodePair ((vsrc / R57 + Jeq) / Gs, 1.0 / Gs, diode);

        const double W  = aW * vU + bW;
        const double vo = (J37 + J29 - vU / Ri - g37 * W) / Gf;
        J37 = 2.0 * g37 * W - J37;
        J29 = 2.0 * g29 * vo - J29;
        return vo;
    }

    double clipNodeVoltage() const { return vU; }

private:
    double g37 = 0.0, g29 = 0.0, J37 = 0.0, J29 = 0.0, vU = 0.0;
    double G1 = 1.0 / 51e3, G2 = 1.0 / 54.7e3;
};

//==============================================================================
class DS2Model
{
public:
    struct Config
    {
        // Level mapping
        double voltsPerUnitIn  = 1.0;      // [assumed] 0 dBFS at the input = 1 V peak
        double unitsPerVoltOut = 1.0;

        // Input buffer coupling
        double C12 = 47e-9, R16 = 1.0e6;

        // D14/D15 + Q22 booster
        double R62 = 4.7e3;
        Diode  clampDiode;                                // 1SS188FM, treated as silicon [assumed]
        double C40 = 47e-9, R63 = 100e3;
        double R65 = 2.2e3, R66 = 1.5e3, R67 = 220.0, C42 = 10e-6;
        double q22Beta = 300.0;                           // [assumed] GR rank is 200..400
        double q22re = 14.0;                              // ~1.8 mA from the R63 bias
        double q22Load = 25e3;                            // [assumed] what the collector drives
        double q22SwingUp = 3.8, q22SwingDown = 1.2;      // to cutoff / to saturation, V

        // TURBO I path
        double C39 = 47e-9, R61 = 100e3, R49 = 100e3;

        // TURBO II path
        double C41 = 2.2e-9, R64 = 10e3, R68 = 10e3, R71 = 100e3, C45 = 22e-9;
        double R69 = 22e3, R70 = 22e3, C43 = 47e-9, C44 = 1e-9;
        double q23Gain = 0.99;                            // [assumed] emitter-follower gain (sets Q)

        // Discrete op-amp
        double R54 = 1e3, C33 = 0.22e-6, C32 = 100e-12;
        double R55 = 1e3, VR1 = 250e3;
        double distTaperMid = 0.10;                       // [assumed] "D" taper, 10% at noon
        double C34 = 2.2e-9, R56 = 1e6;
        Diode  feedbackDiode;
        double opAmpGBW = 1.0e6;                          // [assumed]
        double opAmpSwingUp = 3.0, opAmpSwingDown = 2.3;  // [assumed] Q17 headroom / R52 sink limit

        // Hard clipper + tone stage
        double C35 = 10e-6, R57 = 4.7e3;
        Diode  clipDiode;
        double R58 = 15e3, R46 = 33e3, R59 = 1e3, R45 = 4.7e3, VR2 = 100e3, C37 = 6.8e-9, C29 = 470e-12;

        // Bridged-T (R44 = R33) and the shelf network after Q12
        double R44 = 22e3, C28 = 4.7e-9, C21 = 10e-9;
        double R42 = 10e3, C26 = 1e-9, R41 = 10e3, C25 = 6.8e-9;

        // LEVEL
        double VR3 = 50e3, levelTaperMid = 0.15, levelLoad = 50e3;   // load [assumed]

        // What the TURBO switch changes (traced from Q8/Q9; flip to experiment)
        bool turbo1FeedbackDiodes = true;   // Q20
        bool turbo1BridgedT       = true;   // Q10
        bool turbo2Q23Path        = true;   // Q15 (vs Q14)

        double smoothingMs = 30.0;
        double modeFadeMs  = 6.0;
    };

    DS2Model() = default;
    explicit DS2Model (const Config& c) : cfg (c) {}

    Config& config() { return cfg; }   // call prepare() again after editing

    //--------------------------------------------------------------------------
    void prepare (double sampleRate)
    {
        fs = sampleRate;
        const Config& c = cfg;

        inHP.setHighpass (c.C12 * c.R16, fs);

        // Q22 input impedance: R63 || beta * (re + R66 || R67)
        const double reAC   = c.R66 * c.R67 / (c.R66 + c.R67);
        const double rBase  = c.q22Beta * (c.q22re + reAC);
        const double rIn    = c.R63 * rBase / (c.R63 + rBase);
        clampDiv = rIn / (c.R62 + rIn);
        clampRth = c.R62 * rIn / (c.R62 + rIn);
        q22HP.setHighpass (c.C40 * (clampRth + rIn), fs);
        {
            const double Rc = c.R65 * c.q22Load / (c.R65 + c.q22Load);
            const double Re = c.R66, R67 = c.R67, C = c.C42, re = c.q22re;
            q22Shelf.setAnalog (Rc, Rc * C * (Re + R67),
                                re + Re, C * (re * (Re + R67) + Re * R67), fs, 60.0);
        }

        const double rSrc = c.R65;   // collector output impedance

        // TURBO I: C39 into R61 || R49
        c39HP.setHighpass (c.C39 * (rSrc + c.R61 * c.R49 / (c.R61 + c.R49)), fs);

        // TURBO II: Q23 Sallen-Key, then nodes Y (C45/R71) and X (C41-R64, R68, R49)
        {
            const double K = c.q23Gain, R1 = c.R69 + rSrc, R2 = c.R70;
            q23SK.setAnalog (K, 0.0, 0.0,
                             1.0, c.C44 * (R1 + R2) + R1 * c.C43 * (1.0 - K),
                             R1 * R2 * c.C43 * c.C44, fs, 1000.0);

            const double G68 = 1.0 / c.R68, G49 = 1.0 / c.R49, G71 = 1.0 / c.R71;
            const double a   = G68 + G49, t1 = c.C41 * (rSrc + c.R64);
            const double d0  = a * (G68 + G71) - G68 * G68;
            const double d1  = a * c.C45 + (c.C41 + a * t1) * (G68 + G71) - G68 * G68 * t1;
            const double d2  = (c.C41 + a * t1) * c.C45;
            xFromCollector.setAnalog (0.0, c.C41 * (G68 + G71), c.C41 * c.C45, d0, d1, d2, fs, 1000.0);
            xFromQ23.setAnalog (0.0, G68 * c.C45, G68 * c.C45 * t1, d0, d1, d2, fs, 1000.0);
        }

        gain.R54 = c.R54;  gain.C33 = c.C33;  gain.C32 = c.C32;
        gain.C34 = c.C34;  gain.R56 = c.R56;
        gain.gbw = c.opAmpGBW;  gain.swingUp = c.opAmpSwingUp;  gain.swingDown = c.opAmpSwingDown;
        gain.diode = c.feedbackDiode;
        gain.prepare (fs);

        c35HP.setHighpass (c.C35 * (c.R57 + c.R58), fs);

        clipTone.R57 = c.R57;  clipTone.Ri = c.R58;  clipTone.Rf = c.R46;
        clipTone.Ra = c.R59;   clipTone.Rb = c.R45;  clipTone.pot = c.VR2;
        clipTone.C37 = c.C37;  clipTone.C29 = c.C29; clipTone.diode = c.clipDiode;
        clipTone.prepare (fs);

        {
            const double R = c.R44, C1 = c.C28, C2 = c.C21;
            bridgedT.setAnalog (1.0, 2.0 * R * C1, R * R * C1 * C2,
                                1.0, R * (2.0 * C1 + C2), R * R * C1 * C2, fs, 1000.0);
        }
        {
            const double t41 = c.R41 * c.C25, t42 = c.R42 * c.C26, t425 = c.R42 * c.C25;
            outShelf.setAnalog (1.0, t41, 0.0, 1.0, t41 + t42 + t425, t41 * t42, fs, 2000.0);
        }
        outHP.setHighpass (1.0 / (2.0 * kPi * 3.0), fs);

        smoothCoef = 1.0 - std::exp (-1.0 / (0.001 * c.smoothingMs * fs));
        fadeStep   = 1.0 / std::max (1.0, 0.001 * c.modeFadeMs * fs);

        reset();
    }

    void reset()
    {
        inHP.reset();  q22HP.reset();  q22Shelf.reset();
        c39HP.reset(); q23SK.reset();  xFromCollector.reset(); xFromQ23.reset();
        gain.reset();  c35HP.reset();  clipTone.reset();
        bridgedT.reset(); outShelf.reset(); outHP.reset();
        vClamp = 0.0;

        sDist = tDist;  sTone = tTone;  sLevel = tLevel;  sIn = tIn;  sOut = tOut;
        mode2 = wantMode2;  fade = 1.0;  fadingOut = false;
        controlCounter = 0;
        updateControls();
    }

    //--------------------------------------------------------------------------
    // Knobs 0..1 (noon = 0.5). Cheap; call every block from the audio thread.
    void setDist  (float v) { tDist  = clamp01 (v); }
    void setTone  (float v) { tTone  = clamp01 (v); }
    void setLevel (float v) { tLevel = clamp01 (v); }
    void setTurbo (int mode) { wantMode2 = (mode == 2); }
    void setInputTrimDb  (float db) { tIn  = std::pow (10.0, db / 20.0); }
    void setOutputTrimDb (float db) { tOut = std::pow (10.0, db / 20.0); }

    //--------------------------------------------------------------------------
    void process (float* data, int numSamples)
    {
        for (int i = 0; i < numSamples; ++i)
            data[i] = (float) processSample ((double) data[i]);
    }

    double processSample (double x)
    {
        sDist  += smoothCoef * (tDist  - sDist);
        sTone  += smoothCoef * (tTone  - sTone);
        sLevel += smoothCoef * (tLevel - sLevel);
        sIn    += smoothCoef * (tIn    - sIn);
        sOut   += smoothCoef * (tOut   - sOut);
        if (++controlCounter >= kControlInterval)
        {
            controlCounter = 0;
            updateControls();
        }

        // TURBO switch: short dip so the FET switching doesn't click
        if (wantMode2 != mode2 && ! fadingOut)
            fadingOut = true;
        if (fadingOut)
        {
            fade -= fadeStep;
            if (fade <= 0.0)
            {
                fade = 0.0;
                fadingOut = false;
                mode2 = wantMode2;
                gain.resetDiodeBranch();
            }
        }
        else if (fade < 1.0)
        {
            fade = std::min (1.0, fade + fadeStep);
        }

        const Config& c = cfg;

        // input buffer, D14/D15
        const double v = inHP.process (x * c.voltsPerUnitIn * sIn);
        vClamp = solveShuntDiodePair (v * clampDiv, clampRth, c.clampDiode);

        // Q22 (inverting): saturates ~1 V below its bias point, cuts off ~4 V above it
        const double b    = q22Shelf.process (q22HP.process (vClamp));
        const double vq22 = asymLimit (-b, c.q22SwingUp, c.q22SwingDown);

        // TURBO paths (all filters run, so switching is seamless)
        const double pathI  = c39HP.process (vq22);
        const double pathII = xFromCollector.process (vq22) + xFromQ23.process (q23SK.process (vq22));
        const bool   t1     = ! mode2;
        const double vin    = (mode2 && c.turbo2Q23Path) ? pathII : pathI;

        // discrete op-amp, then C35 -> R57 -> D11/D12 + tone stage
        const double vo   = c35HP.process (gain.process (vin, t1 && c.turbo1FeedbackDiodes));
        const double vt   = clipTone.process (vo);

        // bridged-T (Q10 on in TURBO I), Q12 + shelf network, LEVEL
        const double vbt  = bridgedT.process (vt);
        double y = (t1 && c.turbo1BridgedT) ? vbt : vt;
        y = outShelf.process (y) * levelGain;
        y = outHP.process (y);

        // Q22 and Q13 both invert, so the pedal as a whole is non-inverting
        return y * c.unitsPerVoltOut * sOut * fade;
    }

    // Diagnostics
    double currentFeedbackResistance() const { return rDist; }
    double currentLevelGain() const { return levelGain; }
    bool   isTurbo2() const { return mode2; }

private:
    static float clamp01 (float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

    // (e^(k t) - 1) / (e^k - 1), with k chosen so f(0.5) = mid
    static double expTaper (double t, double mid)
    {
        const double k = 2.0 * std::log ((1.0 - mid) / mid);
        return (std::exp (k * t) - 1.0) / (std::exp (k) - 1.0);
    }

    void updateControls()
    {
        const Config& c = cfg;

        rDist = c.R55 + c.VR1 * expTaper (sDist, c.distTaperMid);
        gain.setFeedbackResistance (rDist);

        clipTone.setTone (sTone);

        // LEVEL: signal enters the wiper (from R42), leaves at the MAX end into ~levelLoad.
        const double alpha = expTaper (sLevel, c.levelTaperMid);
        const double rl = std::max (alpha * c.VR3, 1.0);           // wiper -> ground
        const double ru = (1.0 - alpha) * c.VR3;                   // wiper -> MAX end
        const double zw = rl * (ru + c.levelLoad) / (rl + ru + c.levelLoad);
        levelGain = (alpha <= 0.0) ? 0.0
                  : zw / (c.R42 + zw) * c.levelLoad / (ru + c.levelLoad);
    }

    static constexpr int kControlInterval = 32;

    Config cfg;
    double fs = 176400.0;

    FirstOrder    inHP, q22HP, q22Shelf, c39HP, c35HP, outHP;
    Biquad        q23SK, xFromCollector, xFromQ23, bridgedT, outShelf;
    GainStage     gain;
    ClipToneStage clipTone;

    double clampDiv = 1.0, clampRth = 1.0, vClamp = 0.0;

    double tDist = 0.5, tTone = 0.5, tLevel = 0.5, tIn = 1.0, tOut = 1.0;
    double sDist = 0.5, sTone = 0.5, sLevel = 0.5, sIn = 1.0, sOut = 1.0;
    double smoothCoef = 0.001, rDist = 1.0e3, levelGain = 0.2;
    int    controlCounter = 0;

    bool   wantMode2 = false, mode2 = false, fadingOut = false;
    double fade = 1.0, fadeStep = 0.001;
};

} // namespace ds2
