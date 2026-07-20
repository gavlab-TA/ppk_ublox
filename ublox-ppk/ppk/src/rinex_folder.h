// rinex_folder.h
// -----------------------------------------------------------------------------
// Pick the RINEX a session actually needs out of a folder of RINEX.
//
// The field workflow is "download the station's files for the trip and hand the
// whole folder over" -- a week of daily files, hourly files, a mix, gzipped
// and/or Hatanaka-compressed. This resolves that folder against the rover's own
// GPS time span and stages exactly the files that overlap it.
//
// The day rollover is the case that motivates this: a session that starts at
// 23:43 UTC and ends at 03:41 needs *two* daily files merged into one base
// stream. RTKLIB will not do that for you if you hand it two files -- readobsnav
// assigns each input file a new receiver index, so the second day would be
// silently treated as a third receiver. The files that survive selection are
// therefore staged under a single wildcard-matchable name, which readrnxt()
// expands internally and reads under one receiver index; sortobs() then orders
// the merged stream by time.
//
// Selection is by *GPS time*, never by the recording host's wall clock. On the
// data this was built against those disagreed by ~1.7 days, which would have
// silently picked the wrong day's base file.
// -----------------------------------------------------------------------------

#pragma once

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>

extern "C" {
#include "rtklib.h"
}

namespace rinexdir {

struct File {
    std::string path;      // absolute/relative path as found
    std::string name;      // basename
    bool   is_obs = false; // observation vs navigation
    char   navsys = ' ';   // for nav: 'n' gps, 'g' glonass, 'p' mixed, 'l' galileo, ...
    bool   gz = false;     // .gz wrapper
    bool   hatanaka = false;
    int    year = 0, doy = 0;
    double t0 = 0, t1 = 0; // coverage window, GPS seconds since epoch
    std::string station;
};

inline bool ends_with_ci(const std::string& s, const std::string& suf) {
    if (s.size() < suf.size()) return false;
    return std::equal(suf.rbegin(), suf.rend(), s.rbegin(),
                      [](char a, char b) { return std::tolower(a) == std::tolower(b); });
}

inline std::string lower(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::tolower(c));
    return s;
}

// RINEX 2-digit year: 80-99 -> 19xx, 00-79 -> 20xx.
inline int rnx_year(int yy) { return yy >= 80 ? 1900 + yy : 2000 + yy; }

inline gtime_t doy2time(int year, int doy) {
    double ep[6] = {(double)year, 1, 1, 0, 0, 0};
    return timeadd(epoch2time(ep), (doy - 1) * 86400.0);
}

// Classify a filename against the two RINEX naming conventions.
//
//   short (RINEX 2): ssssDDDf.YYt[.gz]      f='0' daily, 'a'..'x' hourly
//   long  (RINEX 3): SSSSMRCCC_K_YYYYDDDHHMM_PPU_FFU_TT.FMT[.gz]
//
// Returns false for anything that is not recognisably RINEX.
inline bool classify(const std::string& dir, const std::string& fname, File& f) {
    f.name = fname;
    f.path = dir + "/" + fname;

    std::string n = lower(fname);
    if (ends_with_ci(n, ".gz")) { f.gz = true; n = n.substr(0, n.size() - 3); }
    else if (ends_with_ci(n, ".z")) { f.gz = true; n = n.substr(0, n.size() - 2); }

    // --- long RINEX 3 name -------------------------------------------------
    // e.g. ALA100USA_R_20261950000_01D_30S_MO.crx
    if (n.size() > 34 && n[9] == '_' && n[11] == '_') {
        int year = std::atoi(n.substr(12, 4).c_str());
        int doy  = std::atoi(n.substr(16, 3).c_str());
        int hh   = std::atoi(n.substr(19, 2).c_str());
        int mm   = std::atoi(n.substr(21, 2).c_str());
        if (year < 1980 || doy < 1 || doy > 366) return false;
        std::string type = n.substr(31, 2);           // "MO", "MN", "GO", ...
        std::string fmt  = n.substr(34);              // ".rnx" / ".crx"
        f.hatanaka = (fmt.find("crx") != std::string::npos);
        f.is_obs   = (type.size() == 2 && type[1] == 'o');
        bool is_nav = (type.size() == 2 && type[1] == 'n');
        if (!f.is_obs && !is_nav) return false;
        if (is_nav) f.navsys = type[0] == 'm' ? 'p' : (char)std::tolower(type[0]);
        f.station = n.substr(0, 4);
        f.year = year; f.doy = doy;
        // period field, e.g. "01D" / "01H"
        std::string per = n.substr(24, 3);
        double span = 86400.0;
        if (per.size() == 3 && per[2] == 'h') span = std::atof(per.substr(0, 2).c_str()) * 3600.0;
        else if (per.size() == 3 && per[2] == 'd') span = std::atof(per.substr(0, 2).c_str()) * 86400.0;
        gtime_t t = timeadd(doy2time(year, doy), hh * 3600.0 + mm * 60.0);
        f.t0 = (double)t.time + t.sec;
        f.t1 = f.t0 + span;
        return true;
    }

    // --- short RINEX 2 name ------------------------------------------------
    // ssssDDDf.YYt   (8 chars, '.', 3 chars)
    size_t dot = n.rfind('.');
    if (dot == std::string::npos || dot < 8 || n.size() - dot != 4) return false;
    std::string stem = n.substr(0, dot);
    std::string ext  = n.substr(dot + 1);
    if (stem.size() < 8) return false;
    if (!std::isdigit((unsigned char)ext[0]) || !std::isdigit((unsigned char)ext[1])) return false;

    int yy = std::atoi(ext.substr(0, 2).c_str());
    char t = (char)std::tolower(ext[2]);
    std::string ddd = stem.substr(4, 3);
    if (!std::isdigit((unsigned char)ddd[0])) return false;
    int doy = std::atoi(ddd.c_str());
    if (doy < 1 || doy > 366) return false;
    char sess = stem[7];

    f.station = stem.substr(0, 4);
    f.year = rnx_year(yy);
    f.doy  = doy;

    if (t == 'o') { f.is_obs = true; }
    else if (t == 'd') { f.is_obs = true; f.hatanaka = true; }
    else if (t == 'n' || t == 'g' || t == 'p' || t == 'l' || t == 'f' || t == 'c' || t == 'j') {
        f.is_obs = false; f.navsys = t;
    } else return false;

    gtime_t t0 = doy2time(f.year, doy);
    double span = 86400.0, off = 0.0;
    if (sess != '0' && sess >= 'a' && sess <= 'x') { off = (sess - 'a') * 3600.0; span = 3600.0; }
    f.t0 = (double)t0.time + t0.sec + off;
    f.t1 = f.t0 + span;
    return true;
}

// ---------------------------- decompression ----------------------------------
// .gz is handled with zlib; Hatanaka needs the RNXCMP crx2rnx binary (the
// `hatanaka` pip package ships one, as the README's download step already
// assumes). Nothing else shells out.
bool gunzip_to(const std::string& in, const std::string& out);
bool crx2rnx_to(const std::string& in, const std::string& out);

// Resolve `in` (possibly .gz and/or Hatanaka) into a plain RINEX file at `out`.
inline bool materialize(const File& f, const std::string& out, std::string& err) {
    std::string src = f.path;
    std::string tmp;
    if (f.gz) {
        tmp = out + (f.hatanaka ? ".crx" : ".tmp");
        if (!gunzip_to(src, tmp)) { err = "gunzip failed: " + src; return false; }
        src = tmp;
    }
    if (f.hatanaka) {
        if (!crx2rnx_to(src, out)) {
            err = "crx2rnx failed: " + src +
                  "\n       Hatanaka-compressed RINEX needs the crx2rnx binary on PATH"
                  "\n       (pip install hatanaka, or build RNXCMP).";
            return false;
        }
        if (!tmp.empty()) std::remove(tmp.c_str());
        return true;
    }
    if (f.gz) { return std::rename(tmp.c_str(), out.c_str()) == 0; }
    // plain file: copy so every staged base shares one wildcard-matchable name
    FILE* a = std::fopen(src.c_str(), "rb");
    if (!a) { err = "cannot open " + src; return false; }
    FILE* b = std::fopen(out.c_str(), "wb");
    if (!b) { std::fclose(a); err = "cannot write " + out; return false; }
    char buf[65536]; size_t r;
    while ((r = std::fread(buf, 1, sizeof(buf), a)) > 0) std::fwrite(buf, 1, r, b);
    std::fclose(a); std::fclose(b);
    return true;
}

} // namespace rinexdir
