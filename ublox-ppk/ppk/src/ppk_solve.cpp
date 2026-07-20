// ppk_solve.cpp
// -----------------------------------------------------------------------------
// Post-processed kinematic (PPK) solver.
//
//   rover (u-blox .ubx  OR  RINEX .obs)
//   + a folder of CORS RINEX  (--base-dir: daily/hourly, .gz and/or Hatanaka)
//   -> cm-level track as a plain CSV (ppk.csv), plus a summary of fixed epochs.
//
// Note this *solves* for position; it does not correct someone else's. The
// engine forms double-differenced carrier-phase observations between the rover
// and the base, resolves the integer ambiguities, and runs its own Kalman
// filter -- the receiver's own reported positions are never an input. Only raw
// observables (pseudorange/carrier phase/Doppler) are.
//
// This is a thin, deterministic wrapper around the demo5 RTKLIB engine: it
// reuses RTKLIB's integer ambiguity resolution (LAMBDA), cycle-slip handling,
// and troposphere model rather than reimplementing them. All tuning lives in
// the .conf file; this program only pins the essentials, resolves which base
// files a session needs, and injects the base station position so results are
// reproducible.
//
// Base input is a *folder*, because that is how CORS data arrives: you download
// the station's files for the trip and hand the whole lot over. ppk_solve reads
// the rover's GPS time span and stages exactly the files that overlap it,
// merging across a UTC day rollover when a session crosses midnight. See
// rinex_folder.h for why the merge cannot simply be "pass RTKLIB two files".
//
// Datum note: the output track is expressed in the reference frame of the base
// station coordinates you supply. Use the CORS published ARP position (ITRF2020
// or NAD83(2011)) from the station's .coord file; the vehicle solution comes
// out in that same frame. The RINEX header's APPROX POSITION is *not* neutral --
// for a NOAA CORS station it is the NAD83(2011) value, which sits ~1.8 m from
// ITRF2020 in North America. If you intend to compare against a receiver's own
// WGS84/ITRF solution, pass --base-ecef with the ITRF position propagated to
// the observation epoch.
//
// Build: see ppk/CMakeLists.txt (fetches + builds RTKLIB demo5).
// -----------------------------------------------------------------------------

#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <zlib.h>

extern "C" {
#include "rtklib.h"
}

#include "rinex_folder.h"

// RTKLIB is a library but expects the *application* to define these three
// callbacks (used for progress/abort). Without them the link fails.
extern "C" int  showmsg(const char* format, ...) {
    va_list ap; va_start(ap, format); vfprintf(stderr, format, ap); va_end(ap);
    fprintf(stderr, "\r"); return 0;
}
extern "C" void settspan(gtime_t, gtime_t) {}
extern "C" void settime (gtime_t)          {}

// ---- decompression helpers declared by rinex_folder.h -----------------------
namespace rinexdir {

bool gunzip_to(const std::string& in, const std::string& out) {
    gzFile g = gzopen(in.c_str(), "rb");
    if (!g) return false;
    FILE* f = std::fopen(out.c_str(), "wb");
    if (!f) { gzclose(g); return false; }
    char buf[65536];
    int n;
    bool ok = true;
    while ((n = gzread(g, buf, sizeof(buf))) > 0)
        if (std::fwrite(buf, 1, (size_t)n, f) != (size_t)n) { ok = false; break; }
    if (n < 0) ok = false;
    gzclose(g);
    std::fclose(f);
    return ok;
}

bool crx2rnx_to(const std::string& in, const std::string& out) {
    // RNXCMP's crx2rnx reads stdin and writes stdout when handed "-".
    const char* names[] = { "crx2rnx", "CRX2RNX" };
    for (const char* nm : names) {
        std::string cmd = std::string(nm) + " - < '" + in + "' > '" + out + "' 2>/dev/null";
        if (std::system(cmd.c_str()) == 0) {
            struct stat st;
            if (stat(out.c_str(), &st) == 0 && st.st_size > 0) return true;
        }
    }
    return false;
}

} // namespace rinexdir

namespace {

bool ends_with_ci(const std::string& s, const std::string& suf) {
    if (s.size() < suf.size()) return false;
    return std::equal(suf.rbegin(), suf.rend(), s.rbegin(),
                      [](char x, char y){ return std::tolower(x) == std::tolower(y); });
}

// Seconds-since-epoch (the form rinex_folder.h computes coverage in) -> gtime_t.
gtime_t sec2time(double s) {
    gtime_t t;
    t.time = (time_t)std::floor(s);
    t.sec  = s - (double)t.time;
    return t;
}

std::string tstr(double s) {
    char b[64];
    gtime_t t = sec2time(s);
    time2str(t, b, 0);
    return b;
}

// Parse "APPROX POSITION XYZ" (ECEF meters) from a RINEX observation header.
// For a CORS station this record carries the published ARP position -- in
// whatever datum the agency publishes (NAD83(2011) for NOAA CORS).
bool rinex_approx_pos(const std::string& obs, double xyz[3]) {
    FILE* fp = std::fopen(obs.c_str(), "r");
    if (!fp) return false;
    char line[512];
    bool ok = false;
    while (std::fgets(line, sizeof(line), fp)) {
        if (std::strstr(line, "APPROX POSITION XYZ")) {
            if (std::sscanf(line, "%lf %lf %lf", &xyz[0], &xyz[1], &xyz[2]) == 3)
                ok = (xyz[0] != 0.0 || xyz[1] != 0.0 || xyz[2] != 0.0);
            break;
        }
        if (std::strstr(line, "END OF HEADER")) break;
    }
    std::fclose(fp);
    return ok;
}

// Convert a u-blox .ubx log to RINEX 3.04 obs + nav via RTKLIB's convrnx().
// rcvopt is handed to RTKLIB's u-blox decoder (see -MAX_STD_CP / -STD_SLIP).
bool ubx_to_rinex(const std::string& ubx, const std::string& rcvopt,
                  std::string& obs_out, std::string& nav_out) {
    rnxopt_t opt; std::memset(&opt, 0, sizeof(opt));
    opt.rnxver   = 304;                                 // RINEX 3.04
    opt.navsys   = SYS_GPS | SYS_GLO | SYS_GAL | SYS_CMP;
    opt.obstype  = OBSTYPE_ALL;
    opt.freqtype = FREQTYPE_ALL;
    opt.tint     = 0.0;
    for (int i = 0; i < RNX_NUMSYS; ++i)
        for (int j = 0; j < MAXCODE; ++j) opt.mask[i][j] = '1';
    std::strcpy(opt.prog, "ppk_solve");
    std::snprintf(opt.rcvopt, sizeof(opt.rcvopt), "%s", rcvopt.c_str());

    obs_out = ubx + ".obs";
    nav_out = ubx + ".nav";

    // convrnx ofile slots: 0=obs 1=nav 2=gnav 3=hnav 4=qnav 5=lnav 6=cnav ...
    static char slot[10][1024];
    char* ofile[10];
    for (int i = 0; i < 10; ++i) { slot[i][0] = '\0'; ofile[i] = slot[i]; }
    std::snprintf(slot[0], sizeof(slot[0]), "%s", obs_out.c_str());
    std::snprintf(slot[1], sizeof(slot[1]), "%s", nav_out.c_str());

    int stat = convrnx(STRFMT_UBX, &opt, ubx.c_str(), ofile);
    if (stat < 1) return false;

    FILE* t = std::fopen(obs_out.c_str(), "r");           // verify obs produced
    if (!t) return false;
    std::fclose(t);
    return true;
}

// Read the rover's observation time span, and how regularly it is sampled.
// The span drives base-file selection, so it comes from the observations
// themselves -- not from a filename, and not from the recording host's clock,
// which can be wrong by days. The spacing drives the dynamics-model warning.
bool rover_span(const std::string& obsfile, gtime_t& ts, gtime_t& te,
                double* gap_med = nullptr, double* gap_p90 = nullptr,
                double* gap_max = nullptr) {
    obs_t obs = {0}; nav_t nav = {0};
    gtime_t t0 = {0};
    if (readrnxt(obsfile.c_str(), 1, t0, t0, 0.0, "", &obs, &nav, NULL) < 0 || obs.n <= 0) {
        freeobs(&obs); freenav(&nav, 0xFF);
        return false;
    }
    sortobs(&obs);
    ts = obs.data[0].time;
    te = obs.data[obs.n - 1].time;

    if (gap_med) {
        std::vector<double> epochs;                     // one entry per epoch
        for (int i = 0; i < obs.n; ++i) {
            double t = (double)obs.data[i].time.time + obs.data[i].time.sec;
            if (epochs.empty() || t - epochs.back() > 1e-3) epochs.push_back(t);
        }
        std::vector<double> gaps;
        for (size_t i = 1; i < epochs.size(); ++i) gaps.push_back(epochs[i] - epochs[i - 1]);
        if (!gaps.empty()) {
            std::sort(gaps.begin(), gaps.end());
            *gap_med = gaps[gaps.size() / 2];
            *gap_p90 = gaps[(size_t)(gaps.size() * 0.9)];
            *gap_max = gaps.back();
        }
    }
    freeobs(&obs); freenav(&nav, 0xFF);
    return true;
}

// Count broadcast ephemerides across the nav files actually selected.
// A u-blox log carries SFRBX, so convrnx yields a populated nav file and this is
// non-zero; an observables-only log yields a nav file with a header and nothing
// in it, which looks identical until you read it.
int nav_count(const std::vector<std::string>& files) {
    nav_t nav = {0}; obs_t obs = {0}; gtime_t z = {0};
    for (const auto& f : files) readrnxt(f.c_str(), 0, z, z, 0.0, "", &obs, &nav, NULL);
    int n = nav.n + nav.ng + nav.ns;
    freeobs(&obs); freenav(&nav, 0xFF);
    return n;
}

// Number of solution rows in a .pos, so a failed solve cannot be reported as a
// success. postpos() returns >= 0 even when it aborts with "no nav data".
long pos_rows(const std::string& pos_file) {
    FILE* fp = std::fopen(pos_file.c_str(), "r");
    if (!fp) return 0;
    char line[1024]; long n = 0;
    while (std::fgets(line, sizeof(line), fp))
        if (line[0] != '%' && line[0] != '#' && std::strlen(line) > 8) ++n;
    std::fclose(fp);
    return n;
}

const char* qual_name(int q) {
    switch (q) {
        case 1: return "fix";    case 2: return "float";  case 3: return "sbas";
        case 4: return "dgps";   case 5: return "single"; case 6: return "ppp";
    }
    return "none";
}

// Rewrite RTKLIB's .pos as a plain CSV: one row per epoch, ECEF and LLA side by
// side, sd rotated into ENU (the .pos carries it in ECEF, which nothing
// downstream wants), and a UTC stamp so the track can be joined against other
// vehicle logs without anyone having to know GPS time.
bool write_csv(const std::string& pos_file, const std::string& csv_file) {
    FILE* fp = std::fopen(pos_file.c_str(), "r");
    if (!fp) { fprintf(stderr, "[csv] cannot read %s\n", pos_file.c_str()); return false; }
    FILE* fo = std::fopen(csv_file.c_str(), "w");
    if (!fo) { std::fclose(fp); fprintf(stderr, "[csv] cannot write %s\n", csv_file.c_str()); return false; }

    std::fprintf(fo,
        "gps_week,gps_tow_s,utc,"
        "lat_deg,lon_deg,height_m,"
        "ecef_x_m,ecef_y_m,ecef_z_m,"
        "q,quality,n_sats,sd_e_m,sd_n_m,sd_u_m,age_s,ar_ratio\n");

    char line[1024];
    long n = 0;
    while (std::fgets(line, sizeof(line), fp)) {
        if (line[0] == '%' || line[0] == '#') continue;
        std::vector<std::string> t;
        char* p = std::strtok(line, " \t\r\n");
        while (p) { t.emplace_back(p); p = std::strtok(nullptr, " \t\r\n"); }
        // XYZ format: week tow x y z Q ns sdx sdy sdz sdxy sdyz sdzx age ratio
        if (t.size() < 15) continue;

        int    week = std::atoi(t[0].c_str());
        double tow  = std::atof(t[1].c_str());
        double xyz[3] = { std::atof(t[2].c_str()), std::atof(t[3].c_str()), std::atof(t[4].c_str()) };
        int    Q  = std::atoi(t[5].c_str());
        int    ns = std::atoi(t[6].c_str());
        double sdx = std::atof(t[7].c_str()),  sdy  = std::atof(t[8].c_str());
        double sdz = std::atof(t[9].c_str()),  sdxy = std::atof(t[10].c_str());
        double sdyz = std::atof(t[11].c_str()), sdzx = std::atof(t[12].c_str());
        double age = std::atof(t[13].c_str()), ratio = std::atof(t[14].c_str());

        double pos[3];
        ecef2pos(xyz, pos);

        // RTKLIB writes off-diagonal terms as a signed square root, so square
        // them back with the sign preserved to rebuild the covariance.
        auto unsq = [](double v) { return v < 0 ? -v * v : v * v; };
        double P[9] = {
            sdx * sdx,   unsq(sdxy),  unsq(sdzx),
            unsq(sdxy),  sdy * sdy,   unsq(sdyz),
            unsq(sdzx),  unsq(sdyz),  sdz * sdz
        };
        double Qenu[9];
        covenu(pos, P, Qenu);
        double sde = std::sqrt(Qenu[0] > 0 ? Qenu[0] : 0.0);
        double sdn = std::sqrt(Qenu[4] > 0 ? Qenu[4] : 0.0);
        double sdu = std::sqrt(Qenu[8] > 0 ? Qenu[8] : 0.0);

        char utc[64];
        time2str(gpst2utc(gpst2time(week, tow)), utc, 3);

        std::fprintf(fo,
            "%d,%.3f,%s,%.9f,%.9f,%.4f,%.4f,%.4f,%.4f,%d,%s,%d,%.4f,%.4f,%.4f,%.2f,%.1f\n",
            week, tow, utc,
            pos[0] * R2D, pos[1] * R2D, pos[2],
            xyz[0], xyz[1], xyz[2],
            Q, qual_name(Q), ns, sde, sdn, sdu, age, ratio);
        ++n;
    }
    std::fclose(fp);
    std::fclose(fo);
    fprintf(stderr, "[csv] wrote %s (%ld rows)\n", csv_file.c_str(), n);
    return true;
}

// Summarize the .pos: fix rate, and last fixed epoch in ECEF + LLA.
void summarize(const std::string& pos_file) {
    FILE* fp = std::fopen(pos_file.c_str(), "r");
    if (!fp) { fprintf(stderr, "[warn] cannot open %s for summary\n", pos_file.c_str()); return; }

    char line[1024];
    long total = 0, fixed = 0, floatn = 0;
    double last_fixed[3] = {0, 0, 0};
    double sum_fixed[3]  = {0, 0, 0};
    bool have_fixed = false;

    while (std::fgets(line, sizeof(line), fp)) {
        if (line[0] == '%' || line[0] == '#') continue;
        // XYZ format tokens: <t1> <t2> x y z Q ns ...   (time is always 2 tokens)
        std::vector<std::string> tok;
        char* p = std::strtok(line, " \t\r\n");
        while (p) { tok.emplace_back(p); p = std::strtok(nullptr, " \t\r\n"); }
        if (tok.size() < 6) continue;
        double x = std::atof(tok[2].c_str());
        double y = std::atof(tok[3].c_str());
        double z = std::atof(tok[4].c_str());
        int    Q = std::atoi(tok[5].c_str());
        ++total;
        if (Q == 1) {
            ++fixed; have_fixed = true;
            last_fixed[0] = x; last_fixed[1] = y; last_fixed[2] = z;
            sum_fixed[0] += x; sum_fixed[1] += y; sum_fixed[2] += z;
        } else if (Q == 2) {
            ++floatn;
        }
    }
    std::fclose(fp);

    fprintf(stderr, "\n================ PPK summary ================\n");
    fprintf(stderr, "epochs         : %ld\n", total);
    if (total > 0)
        fprintf(stderr, "fixed (Q=1)    : %ld  (%.1f%%)\n", fixed, 100.0 * fixed / total);
    fprintf(stderr, "float (Q=2)    : %ld\n", floatn);

    auto report = [](const char* label, const double xyz[3]) {
        double pos[3];
        ecef2pos(xyz, pos);                              // pos = {lat,lon(rad), h(m)}
        fprintf(stderr, "%s\n", label);
        fprintf(stderr, "  ECEF (m) : X=%.4f  Y=%.4f  Z=%.4f\n", xyz[0], xyz[1], xyz[2]);
        fprintf(stderr, "  LLA      : lat=%.9f deg  lon=%.9f deg  h=%.4f m\n",
                pos[0] * R2D, pos[1] * R2D, pos[2]);
    };

    if (have_fixed) {
        double mean[3] = { sum_fixed[0]/fixed, sum_fixed[1]/fixed, sum_fixed[2]/fixed };
        report("last fixed epoch:", last_fixed);
        report("mean of fixed epochs:", mean);
    } else {
        fprintf(stderr, "[warn] no fixed epochs. Try a nearer/1-Hz base, lower elmask,\n"
                        "       longer occupation, or confirm dual-freq multi-GNSS logging.\n");
    }
}

void usage(const char* p) {
    fprintf(stderr,
      "Usage: %s --rover FILE(.ubx|.obs) (--base-dir DIR | --base BASE.obs)\n"
      "          [--nav BRDC.nav] [--conf f9p_kinematic.conf] [--out ppk.csv]\n"
      "          [--base-ecef X Y Z | --base-llh LAT LON H] [--station SSSS]\n"
      "          [--workdir DIR]\n\n"
      "  --rover      u-blox .ubx log (auto-converted) or a RINEX .obs\n"
      "  --base-dir   folder of CORS RINEX: daily and/or hourly, .gz and/or\n"
      "               Hatanaka (.YYd/.crx). The files overlapping the rover's\n"
      "               GPS time span are selected and merged, across a UTC day\n"
      "               rollover if the session crosses midnight. Broadcast nav\n"
      "               (brdc*.YYn/.YYg) in the same folder is picked up too.\n"
      "  --station    4-char station id, if the folder holds more than one\n"
      "  --base       explicit single base RINEX obs (bypasses --base-dir)\n"
      "  --nav        explicit broadcast nav; optional when --base-dir supplies it\n"
      "  --conf       RTKLIB options file (defaults tuned for F9P kinematic PPK)\n"
      "  --out        PPK track. A .csv name (the default) writes the CSV as the\n"
      "               product and keeps RTKLIB's native .pos in --workdir; any other\n"
      "               name writes the .pos there and a .csv beside it.\n"
      "  --raw-out    uncorrected track: the same observables solved single-point,\n"
      "               so the pair isolates what the corrections bought. Defaults to\n"
      "               --out with 'ppk'->'raw' (or '_raw' before the extension).\n"
      "  --no-raw     skip the uncorrected solve\n"
      "  --base-ecef  base ARP position, ECEF meters (from the CORS .coord file)\n"
      "  --base-llh   base ARP position, lat/lon deg + ellipsoidal height m\n"
      "  --workdir    where staged RINEX and intermediates go (default: next to --out)\n"
      "  --rcvopt     options for RTKLIB's u-blox decoder, e.g.\n"
      "               '-MAX_STD_CP=15 -STD_SLIP=16' to stop a saturated cpStdev\n"
      "               field from being read as a cycle slip\n"
      "  If no base position is given, the base RINEX header APPROX POSITION is\n"
      "  used -- note that is NAD83(2011) for NOAA CORS, not ITRF/WGS84.\n", p);
}

std::string dirname_of(const std::string& p) {
    size_t s = p.rfind('/');
    return s == std::string::npos ? std::string(".") : p.substr(0, s);
}

} // namespace

int main(int argc, char** argv) {
    std::string rover, base, base_dir, nav, conf, out = "ppk_positions.csv";
    std::string station, workdir, rcvopt, raw_out;
    bool have_bpos = false, bpos_llh = false, no_raw = false;
    double bpos[3] = {0, 0, 0};

    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto need = [&](int cnt)->bool { return i + cnt < argc; };
        if      (k == "--rover"    && need(1)) rover    = argv[++i];
        else if (k == "--base"     && need(1)) base     = argv[++i];
        else if (k == "--base-dir" && need(1)) base_dir = argv[++i];
        else if (k == "--nav"      && need(1)) nav      = argv[++i];
        else if (k == "--conf"     && need(1)) conf     = argv[++i];
        else if (k == "--out"      && need(1)) out      = argv[++i];
        else if (k == "--station"  && need(1)) station  = rinexdir::lower(argv[++i]);
        else if (k == "--workdir"  && need(1)) workdir  = argv[++i];
        else if (k == "--rcvopt"   && need(1)) rcvopt   = argv[++i];
        else if (k == "--raw-out"  && need(1)) raw_out  = argv[++i];
        else if (k == "--no-raw")             no_raw   = true;
        else if (k == "--base-ecef" && need(3)) {
            bpos[0]=std::atof(argv[++i]); bpos[1]=std::atof(argv[++i]); bpos[2]=std::atof(argv[++i]);
            have_bpos = true; bpos_llh = false;
        }
        else if (k == "--base-llh" && need(3)) {
            bpos[0]=std::atof(argv[++i]); bpos[1]=std::atof(argv[++i]); bpos[2]=std::atof(argv[++i]);
            have_bpos = true; bpos_llh = true;
        }
        else if (k == "-h" || k == "--help") { usage(argv[0]); return 0; }
        else { fprintf(stderr, "unknown/incomplete arg: %s\n", k.c_str()); usage(argv[0]); return 2; }
    }
    if (rover.empty() || (base.empty() && base_dir.empty())) { usage(argv[0]); return 2; }

    // ---- outputs: CSV is the product; the .pos is an intermediate -----------
    if (workdir.empty()) workdir = dirname_of(out) + "/ppk_work";
    mkdir(workdir.c_str(), 0755);
    bool csv_out = ends_with_ci(out, ".csv");
    std::string pos_path = csv_out ? workdir + "/ppk.pos" : out;
    std::string csv_path = out;
    if (!csv_out) {
        size_t dot = out.rfind('.');
        csv_path = (dot == std::string::npos ? out : out.substr(0, dot)) + ".csv";
    }
    // The uncorrected companion track. "ppk" -> "raw" when the name says ppk,
    // otherwise "_raw" before the extension, so the pair always reads as a pair.
    if (raw_out.empty()) {
        raw_out = csv_path;
        size_t k = raw_out.rfind("ppk");
        if (k != std::string::npos) {
            raw_out.replace(k, 3, "raw");
        } else {
            size_t dot = raw_out.rfind('.');
            raw_out.insert(dot == std::string::npos ? raw_out.size() : dot, "_raw");
        }
    }
    std::string raw_pos_path = workdir + "/raw.pos";

    // ---- options: defaults, optional .conf overrides, then pin essentials ---
    resetsysopts();
    if (!conf.empty() && !loadopts(conf.c_str(), sysopts)) {
        fprintf(stderr, "failed to load conf: %s\n", conf.c_str());
        return 1;
    }
    prcopt_t prcopt; solopt_t solopt; filopt_t filopt;
    std::memset(&filopt, 0, sizeof(filopt));
    getsysopts(&prcopt, &solopt, &filopt);

    prcopt.mode = PMODE_KINEMA;                                   // kinematic rover
    if (prcopt.nf < 2)     prcopt.nf = 2;                         // dual frequency
    if (!prcopt.navsys)    prcopt.navsys = SYS_GPS|SYS_GLO|SYS_GAL|SYS_CMP;
    solopt.posf = SOLF_XYZ;                                       // ECEF .pos (deterministic to parse)
    solopt.height = 0;                                           // ellipsoidal

    // ---- rover: convert .ubx -> RINEX if needed ----------------------------
    std::string rover_obs = rover, gen_nav;
    if (ends_with_ci(rover, ".ubx")) {
        fprintf(stderr, "[conv] %s -> RINEX ...%s%s\n", rover.c_str(),
                rcvopt.empty() ? "" : "  rcvopt: ", rcvopt.c_str());
        if (!ubx_to_rinex(rover, rcvopt, rover_obs, gen_nav)) {
            fprintf(stderr, "ubx->rinex conversion failed\n"); return 1;
        }
        fprintf(stderr, "[conv] obs=%s nav=%s\n", rover_obs.c_str(), gen_nav.c_str());
    }

    // ---- resolve the base folder against the rover's GPS time span ---------
    std::vector<std::string> nav_files;
    if (!nav.empty()) nav_files.push_back(nav);
    std::string base_arg = base;

    // Solve window. Left zeroed (= no limit, the engine's own default) unless the
    // base folder falls short of the session, in which case it clamps the solve
    // to the stretch the corrections actually cover.
    gtime_t solve_ts = {0}, solve_te = {0};
    double  miss_head = 0.0, miss_tail = 0.0;
    bool    coverage_short = false;

    if (!base_dir.empty()) {
        gtime_t ts, te;
        double gmed = 0, gp90 = 0, gmax = 0;
        if (!rover_span(rover_obs, ts, te, &gmed, &gp90, &gmax)) {
            fprintf(stderr, "cannot read rover observations: %s\n", rover_obs.c_str());
            return 1;
        }
        char s1[64], s2[64];
        time2str(ts, s1, 0); time2str(te, s2, 0);
        fprintf(stderr, "[rover] GPS span %s -> %s\n", s1, s2);
        fprintf(stderr, "[rover] epoch spacing: median %.1f s, p90 %.1f s, max %.0f s\n",
                gmed, gp90, gmax);

        // The dynamics model predicts each epoch from the last with a constant
        // acceleration. Across a multi-second gap on a turning vehicle that
        // prediction is simply wrong, and the fix rate collapses -- to zero at
        // 10 s spacing on the data this was built against. Say so; it is a cliff,
        // not a gradual cost, and the symptom (a noisy float solution) looks
        // nothing like the cause.
        if (prcopt.dynamics && gp90 > 5.0) {
            fprintf(stderr,
                "\n[warn] pos1-dynamics=on, but 10%% of epochs are more than %.0f s apart.\n"
                "       The dynamics model assumes epochs close enough to extrapolate\n"
                "       between; it does not degrade gracefully past ~4 s -- expect the\n"
                "       fix rate to collapse. Set pos1-dynamics=off in the conf.\n\n", gp90);
        }

        double rt0 = (double)ts.time + ts.sec, rt1 = (double)te.time + te.sec;

        DIR* dp = opendir(base_dir.c_str());
        if (!dp) { fprintf(stderr, "cannot open --base-dir %s\n", base_dir.c_str()); return 1; }
        std::vector<rinexdir::File> obs_sel, nav_sel;
        int scanned = 0, recognised = 0;
        std::vector<std::string> stations;
        struct dirent* de;
        while ((de = readdir(dp))) {
            if (de->d_name[0] == '.') continue;
            scanned++;
            rinexdir::File f;
            if (!rinexdir::classify(base_dir, de->d_name, f)) continue;
            recognised++;
            // Overlap test, with a margin so a base epoch just outside the rover
            // span is still available to bound interpolation at the extremes.
            if (f.t1 < rt0 - 120.0 || f.t0 > rt1 + 120.0) continue;
            if (f.is_obs) {
                if (!station.empty() && f.station != station) continue;
                obs_sel.push_back(f);
                if (std::find(stations.begin(), stations.end(), f.station) == stations.end())
                    stations.push_back(f.station);
            } else {
                nav_sel.push_back(f);
            }
        }
        closedir(dp);

        fprintf(stderr, "[base-dir] %s: %d files, %d recognised as RINEX\n",
                base_dir.c_str(), scanned, recognised);

        if (stations.size() > 1) {
            fprintf(stderr, "[base-dir] more than one station overlaps the session:");
            for (auto& s : stations) fprintf(stderr, " %s", s.c_str());
            fprintf(stderr, "\n           re-run with --station SSSS to pick one.\n");
            return 1;
        }
        if (obs_sel.empty()) {
            fprintf(stderr, "[base-dir] no base observation file overlaps the rover span.\n"
                            "           The folder holds RINEX, but none of it covers %s..%s.\n"
                            "           (Selection is by GPS time -- check which days you have.)\n",
                    s1, s2);
            return 1;
        }

        // Deterministic order; also makes the log read chronologically.
        auto bytime = [](const rinexdir::File& a, const rinexdir::File& b) { return a.t0 < b.t0; };
        std::sort(obs_sel.begin(), obs_sel.end(), bytime);
        std::sort(nav_sel.begin(), nav_sel.end(), bytime);

        // ---- how much of the session the folder can actually correct --------
        // Overlapping the rover is not the same as covering it. The next day's
        // file routinely does not exist yet -- CORS publishes a daily file only
        // once the UTC day closes, so a session that runs past midnight is
        // downloaded with its own last hours missing -- and an hour can be
        // absent from the middle of a download for the same kind of reason.
        //
        // Corrections cannot be invented for those epochs. Rather than hand the
        // engine a span the base cannot support (RTKLIB would emit single-point
        // epochs for the uncovered tail, which look like solutions and are not),
        // the solve window is clamped to the first contiguous run of base
        // coverage and the shortfall is reported. The corrections file simply
        // ends where the corrections do.
        //
        // Adjacent files are joined: a day rollover is contiguous by
        // construction (day N ends exactly where day N+1 begins), as is hour w
        // -> hour x -> next day. So a rollover is *not* a gap, and the merge
        // below leaves it as one run.
        struct Cover { double t0, t1; };
        std::vector<Cover> runs;
        for (const auto& f : obs_sel) {
            if (!runs.empty() && f.t0 <= runs.back().t1 + 1.0)
                runs.back().t1 = std::max(runs.back().t1, f.t1);
            else
                runs.push_back({ f.t0, f.t1 });
        }

        // The run the session starts in -- corrections run from here until the
        // first hole, and that is where the output ends.
        const Cover* use = nullptr;
        for (const auto& r : runs)
            if (r.t1 > rt0 && r.t0 < rt1) { use = &r; break; }
        if (!use) {
            fprintf(stderr, "[base-dir] no base observation file covers the rover span.\n"
                            "           Files were found near it but none overlaps %s..%s.\n",
                    s1, s2);
            return 1;
        }

        miss_head = use->t0 > rt0 ? use->t0 - rt0 : 0.0;
        miss_tail = use->t1 < rt1 ? rt1 - use->t1 : 0.0;
        coverage_short = (miss_head > 1.0 || miss_tail > 1.0);

        if (coverage_short) {
            double have = std::min(rt1, use->t1) - std::max(rt0, use->t0);
            fprintf(stderr,
                "\n[base-dir] ***** MISSING CORRECTIONS *****\n"
                "           The base folder does not cover the whole session.\n"
                "           session : %s -> %s  (%.0f s)\n"
                "           base    : %s -> %s\n",
                s1, s2, rt1 - rt0, tstr(use->t0).c_str(), tstr(use->t1).c_str());
            if (miss_head > 1.0)
                fprintf(stderr,
                    "           MISSING %.0f s at the START (%s -> %s) -- not solved.\n",
                    miss_head, s1, tstr(use->t0).c_str());
            if (miss_tail > 1.0)
                fprintf(stderr,
                    "           MISSING %.0f s at the END   (%s -> %s) -- not solved.\n",
                    miss_tail, tstr(use->t1).c_str(), s2);
            for (const auto& r : runs) {
                if (&r == use || r.t1 <= rt0 || r.t0 >= rt1) continue;
                fprintf(stderr,
                    "           (a further covered stretch %s -> %s exists after the hole;\n"
                    "            it is NOT used -- fill the gap to solve across it.)\n",
                    tstr(r.t0).c_str(), tstr(r.t1).c_str());
            }
            fprintf(stderr,
                "           Solving %.0f s of %.0f (%.0f%%). The output ends where the\n"
                "           corrections do -- it is not truncated data, it is all the data\n"
                "           the base can support.\n"
                "           Hourly files (ssssDDDh.YYd, h=a..x) publish ~1 h after the hour\n"
                "           and cover this; the daily file only after the UTC day closes.\n\n",
                have, rt1 - rt0, 100.0 * have / (rt1 - rt0));

            // Clamp the engine to the covered window. Left unset when coverage is
            // complete, so a fully-covered session solves exactly as it did before.
            solve_ts = sec2time(std::max(rt0, use->t0));
            solve_te = sec2time(std::min(rt1, use->t1));

            // Only stage what is actually used, so the wildcard cannot pull a
            // post-hole file back into the merged stream.
            std::vector<rinexdir::File> keep;
            for (auto& f : obs_sel)
                if (f.t1 > use->t0 - 1.0 && f.t0 < use->t1 + 1.0) keep.push_back(f);
            obs_sel.swap(keep);
        }

        // A stale base_*.obs from an earlier run with different days would be
        // picked up by the wildcard below and silently rejoin the stream --
        // exactly the coverage this just decided the session does not have.
        if (DIR* wd = opendir(workdir.c_str())) {
            struct dirent* we;
            while ((we = readdir(wd)))
                if (!std::strncmp(we->d_name, "base_", 5))
                    std::remove((workdir + "/" + we->d_name).c_str());
            closedir(wd);
        }

        // Stage every selected base obs under one wildcard-matchable name, so
        // readrnxt() expands them into a single receiver index (see header).
        fprintf(stderr, "[base-dir] station %s, %zu obs file(s) cover the session:\n",
                obs_sel[0].station.c_str(), obs_sel.size());
        for (size_t i = 0; i < obs_sel.size(); ++i) {
            char dst[1024];
            std::snprintf(dst, sizeof(dst), "%s/base_%04d%03d_%02zu.obs",
                          workdir.c_str(), obs_sel[i].year, obs_sel[i].doy, i);
            std::string err;
            if (!rinexdir::materialize(obs_sel[i], dst, err)) {
                fprintf(stderr, "[base-dir] %s\n", err.c_str());
                return 1;
            }
            fprintf(stderr, "           %-28s (%d/%03d) -> %s\n",
                    obs_sel[i].name.c_str(), obs_sel[i].year, obs_sel[i].doy, dst);
        }
        if (obs_sel.size() > 1)
            fprintf(stderr, "[base-dir] session spans %zu RINEX files -- merging into one base\n"
                            "           stream (day rollover).\n", obs_sel.size());

        base_arg = workdir + "/base_*.obs";

        for (size_t i = 0; i < nav_sel.size(); ++i) {
            char dst[1024];
            std::snprintf(dst, sizeof(dst), "%s/nav_%04d%03d_%02zu.%02d%c",
                          workdir.c_str(), nav_sel[i].year, nav_sel[i].doy, i,
                          nav_sel[i].year % 100, nav_sel[i].navsys);
            std::string err;
            if (!rinexdir::materialize(nav_sel[i], dst, err)) {
                fprintf(stderr, "[base-dir] %s\n", err.c_str());
                return 1;
            }
            fprintf(stderr, "[base-dir] nav %-24s -> %s\n", nav_sel[i].name.c_str(), dst);
            nav_files.push_back(dst);
        }
        if (nav_sel.empty() && nav.empty()) {
            fprintf(stderr,
                "[base-dir] no broadcast nav (brdc*.YYn/.YYg) found in the folder.\n");
            if (!gen_nav.empty()) {
                fprintf(stderr, "           falling back to nav decoded from the rover log.\n");
                nav_files.push_back(gen_nav);
            } else {
                fprintf(stderr, "           supply --nav, or add brdc files to the folder.\n");
                return 1;
            }
        }
    } else if (nav.empty() && !gen_nav.empty()) {
        nav_files.push_back(gen_nav);   // explicit --base: rover-decoded nav
    }

    // ---- base station position (always set rb explicitly) ------------------
    double xyz[3];
    std::string hdr_probe = base_arg;
    if (!base_dir.empty()) {
        // A wildcard cannot be opened directly; probe the first staged file.
        DIR* dp = opendir(workdir.c_str());
        if (dp) {
            struct dirent* de;
            std::vector<std::string> hits;
            while ((de = readdir(dp)))
                if (!std::strncmp(de->d_name, "base_", 5)) hits.push_back(workdir + "/" + de->d_name);
            closedir(dp);
            std::sort(hits.begin(), hits.end());
            if (!hits.empty()) hdr_probe = hits.front();
        }
    }
    if (have_bpos) {
        if (bpos_llh) { double p[3] = { bpos[0]*D2R, bpos[1]*D2R, bpos[2] }; pos2ecef(p, xyz); }
        else          { std::memcpy(xyz, bpos, sizeof(xyz)); }
    } else if (!rinex_approx_pos(hdr_probe, xyz)) {
        fprintf(stderr, "no --base-ecef/--base-llh given and base header has no APPROX POSITION;\n"
                        "supply the CORS published ARP position explicitly.\n");
        return 1;
    } else {
        fprintf(stderr, "[base] using RINEX header APPROX POSITION -- for NOAA CORS that is the\n"
                        "       NAD83(2011) value, ~1.8 m from ITRF2020. Pass --base-ecef if the\n"
                        "       track needs to be in a particular frame.\n");
    }
    prcopt.rb[0] = xyz[0]; prcopt.rb[1] = xyz[1]; prcopt.rb[2] = xyz[2];
    double bp[3]; ecef2pos(xyz, bp);
    fprintf(stderr, "[base] ECEF %.3f %.3f %.3f  (lat %.7f lon %.7f h %.3f)\n",
            xyz[0], xyz[1], xyz[2], bp[0]*R2D, bp[1]*R2D, bp[2]);

    // ---- inputs: rover obs, base obs (possibly a wildcard), nav ------------
    std::vector<std::string> files = { rover_obs, base_arg };
    for (auto& nf : nav_files) files.push_back(nf);
    if (files.size() > 8) {
        fprintf(stderr, "[warn] %zu input files; RTKLIB takes at most 8 -- extra nav dropped.\n",
                files.size());
        files.resize(8);
    }
    const char* infile[8]; int n = 0;
    for (auto& f : files) infile[n++] = f.c_str();

    // Fail early and say why. Without this the engine aborts with a bare
    // "no nav data" and still exits 0, which reads as success to a script.
    int neph = nav_count(nav_files);
    if (neph <= 0) {
        fprintf(stderr,
            "\nno broadcast ephemeris in any nav source -- cannot solve.\n"
            "  nav files tried : %zu\n", nav_files.size());
        for (auto& nf : nav_files) fprintf(stderr, "    %s\n", nf.c_str());
        if (!gen_nav.empty() && nav_files.size() == 1 && nav_files[0] == gen_nav)
            fprintf(stderr,
                "  The only nav source was the rover log itself, and it decoded no\n"
                "  ephemeris. A u-blox log carries SFRBX and normally would; an\n"
                "  observables-only log never will.\n");
        fprintf(stderr,
            "  Fix: put broadcast nav for these days in the --base-dir folder\n"
            "       (brdc%s0.YYn for GPS and brdc%s0.YYg for GLONASS, from\n"
            "        https://noaa-cors-pds.s3.amazonaws.com/rinex/YYYY/DDD/),\n"
            "       or pass --nav explicitly.\n", "DDD", "DDD");
        return 1;
    }
    fprintf(stderr, "[solve] postpos: rover=%s base=%s nav=%zu file(s), %d ephemerides\n",
            rover_obs.c_str(), base_arg.c_str(), nav_files.size(), neph);

    int stat = postpos(solve_ts, solve_te, 0.0, 0.0, &prcopt, &solopt, &filopt, infile, n,
                       pos_path.c_str(), "", "");
    if (stat < 0) { fprintf(stderr, "postpos failed (%d)\n", stat); return 1; }
    if (pos_rows(pos_path) == 0) {
        fprintf(stderr, "\nthe solve produced no epochs -- see the engine's error above.\n"
                        "(rover and base may not overlap in time, or nav may not cover them)\n");
        return 1;
    }

    if (!write_csv(pos_path, csv_path)) return 1;

    // ---- uncorrected companion track ---------------------------------------
    // The same observables and the same engine, minus the base: a single-point
    // solution. That isolates exactly what the corrections bought, and it works
    // for any receiver -- a u-blox log has no computed position of its own to
    // compare against, only observables.
    //
    // This is NOT a stand-in for a receiver's real-time output. It is plain SPP
    // on broadcast ephemeris/iono (metre-level). A receiver reporting SBAS/WAAS
    // is applying corrections that are not in the observable log at all, so it
    // will be better than this.
    if (!no_raw) {
        prcopt_t rawopt = prcopt;
        rawopt.mode    = PMODE_SINGLE;
        rawopt.soltype = 0;              // forward only; SPP keeps no filter state
        std::vector<std::string> rfiles = { rover_obs };
        for (auto& nf : nav_files) rfiles.push_back(nf);
        if (rfiles.size() > 8) rfiles.resize(8);
        const char* rinfile[8]; int rn = 0;
        for (auto& f : rfiles) rinfile[rn++] = f.c_str();

        fprintf(stderr, "\n[solve] single-point (uncorrected) from the same observables ...\n");
        gtime_t z = {0};
        if (postpos(z, z, 0.0, 0.0, &rawopt, &solopt, &filopt, rinfile, rn,
                    raw_pos_path.c_str(), "", "") < 0 || pos_rows(raw_pos_path) == 0) {
            fprintf(stderr, "[warn] single-point solve produced nothing; no %s written\n",
                    raw_out.c_str());
            no_raw = true;
        } else if (!write_csv(raw_pos_path, raw_out)) {
            no_raw = true;
        }
    }

    summarize(pos_path);
    // Say it again next to the file paths. The warning above scrolls past a
    // conversion log; this is the line someone reads before using the CSV.
    if (coverage_short) {
        fprintf(stderr, "\n*** MISSING CORRECTIONS -- %s ends early ***\n",
                csv_path.c_str());
        if (miss_head > 1.0)
            fprintf(stderr, "    no base for the first %.0f s of the session\n", miss_head);
        if (miss_tail > 1.0)
            fprintf(stderr, "    no base for the last  %.0f s of the session\n", miss_tail);
        if (!no_raw)
            fprintf(stderr, "    %s still spans the whole session (it needs no base).\n",
                    raw_out.c_str());
    }
    fprintf(stderr, "ppk positions  : %s%s\n", csv_path.c_str(),
            coverage_short ? "   (SHORT -- see above)" : "");
    if (!no_raw)
        fprintf(stderr, "raw positions  : %s  (single-point, no corrections)\n",
                raw_out.c_str());
    fprintf(stderr, "rtklib native  : %s\n", pos_path.c_str());
    fprintf(stderr, "============================================\n");
    return 0;
}
