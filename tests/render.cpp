// Offline renderer for DS2Model: raw float32 mono in -> raw float32 mono out.
// Feed it audio that is ALREADY at the oversampled rate (the analysis script does that).
//
//   render in.f32 out.f32 <fs> <dist> <tone> <level> <turbo 1|2> [inTrimDb] [flags]
//   flags: noQ23 (Turbo II uses the Turbo I path), noDiodes / noNotch (drop D8-D9 / bridged-T from Turbo I),
//          toggle=<sec>, sweepDist

#include "../Source/DS2Model.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

int main (int argc, char** argv)
{
    if (argc < 8)
    {
        std::fprintf (stderr, "usage: render in.f32 out.f32 fs dist tone level turbo [inTrimDb] [flags]\n");
        return 1;
    }

    std::FILE* fi = std::fopen (argv[1], "rb");
    if (! fi) { std::perror ("input"); return 1; }
    std::vector<float> buf;
    float tmp[4096];
    size_t n;
    while ((n = std::fread (tmp, sizeof (float), 4096, fi)) > 0)
        buf.insert (buf.end(), tmp, tmp + n);
    std::fclose (fi);

    ds2::DS2Model model;
    for (int a = 9; a < argc; ++a)
    {
        if (! std::strcmp (argv[a], "noQ23"))    model.config().turbo2Q23Path = false;
        if (! std::strcmp (argv[a], "noDiodes")) model.config().turbo1FeedbackDiodes = false;
        if (! std::strcmp (argv[a], "noNotch"))  model.config().turbo1BridgedT = false;
    }

    model.setDist  ((float) std::atof (argv[4]));
    model.setTone  ((float) std::atof (argv[5]));
    model.setLevel ((float) std::atof (argv[6]));
    model.setTurbo (std::atoi (argv[7]));
    model.setInputTrimDb (argc > 8 ? (float) std::atof (argv[8]) : 0.0f);
    model.prepare (std::atof (argv[3]));   // prepare() snaps smoothed params to targets

    // optional automation: toggle=<sec> flips TURBO every <sec>; sweepDist ramps DIST 0->1->0
    double toggleSec = 0.0;
    bool sweepDist = false;
    for (int a = 9; a < argc; ++a)
    {
        if (! std::strncmp (argv[a], "toggle=", 7)) toggleSec = std::atof (argv[a] + 7);
        if (! std::strcmp (argv[a], "sweepDist"))   sweepDist = true;
    }
    const double fs = std::atof (argv[3]);
    int turbo = std::atoi (argv[7]);

    const auto t0 = std::chrono::steady_clock::now();
    const size_t block = 512;
    bool bad = false;
    for (size_t i = 0; i < buf.size(); i += block)
    {
        const size_t len = std::min (block, buf.size() - i);
        const double t = (double) i / fs;
        if (toggleSec > 0.0)
            model.setTurbo (((long) (t / toggleSec)) % 2 == 0 ? turbo : 3 - turbo);
        if (sweepDist)
        {
            const double ph = t / ((double) buf.size() / fs);
            model.setDist ((float) (ph < 0.5 ? 2.0 * ph : 2.0 - 2.0 * ph));
        }
        model.process (buf.data() + i, (int) len);
        for (size_t k = 0; k < len; ++k)
            if (! std::isfinite (buf[i + k])) bad = true;
    }
    const double secs = std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count();

    std::FILE* fo = std::fopen (argv[2], "wb");
    std::fwrite (buf.data(), sizeof (float), buf.size(), fo);
    std::fclose (fo);

    const double audioSecs = (double) buf.size() / std::atof (argv[3]);
    std::fprintf (stderr, "rdist=%.0f ohm  realtime x%.1f (oversampled rate, mono)  %s\n",
                  model.currentFeedbackResistance(), audioSecs / secs, bad ? "NONFINITE!" : "finite");
#ifdef DS2_STATS
    auto& st = ds2::solverStats();
    std::fprintf (stderr, "newton: calls=%lld avg_iters=%.2f max_iters=%d\n",
                  st.calls, st.calls ? (double) st.iters / st.calls : 0.0, st.maxIters);
#endif
    return bad ? 2 : 0;
}
