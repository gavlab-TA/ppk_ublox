// ubx_logger.cpp
// -----------------------------------------------------------------------------
// Minimal, dependency-free logger for a u-blox ZED-F9P (or any F9-gen module).
// It enables the raw-measurement messages needed for PPK/PPP post-processing:
//     UBX-RXM-RAWX   (raw pseudorange + carrier phase + Doppler, all signals)
//     UBX-RXM-SFRBX  (decoded broadcast navigation subframes)
// sets the measurement rate, then streams the raw UBX byte stream verbatim to a
// .ubx file that RTKLIB (demo5) `convrnx`/`convbin` converts directly to RINEX.
//
// Target: Linux / POSIX (termios).
//   - Over USB the F9P appears as a CDC-ACM device, typically /dev/ttyACM0.
//     (Baud rate is nominal on USB CDC; --baud only matters over UART.)
//   - Over UART use e.g. /dev/ttyUSB0 with --port-target uart1 and a high baud.
//
// The config keys below are the *USB* output keys. If you log over UART1,
// pass --port-target uart1 to switch to the UART1 key set.
//
// Build: see logger/CMakeLists.txt. No external dependencies.
// -----------------------------------------------------------------------------

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <cerrno>
#include <string>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {

// ----------------------------- run control -----------------------------------
std::atomic<bool> g_run{true};
void on_sigint(int) { g_run = false; }

// ----------------------------- UBX framing -----------------------------------
constexpr uint8_t UBX_SYNC1 = 0xB5;
constexpr uint8_t UBX_SYNC2 = 0x62;

// 8-bit Fletcher checksum over [class, id, len_lo, len_hi, payload...].
void ubx_checksum(const uint8_t* buf, size_t n, uint8_t& ck_a, uint8_t& ck_b) {
    ck_a = ck_b = 0;
    for (size_t i = 0; i < n; ++i) {
        ck_a = static_cast<uint8_t>(ck_a + buf[i]);
        ck_b = static_cast<uint8_t>(ck_b + ck_a);
    }
}

// Build a complete UBX frame from class/id + payload.
std::vector<uint8_t> ubx_frame(uint8_t cls, uint8_t id,
                               const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> body;
    body.reserve(payload.size() + 4);
    body.push_back(cls);
    body.push_back(id);
    body.push_back(static_cast<uint8_t>(payload.size() & 0xFF));
    body.push_back(static_cast<uint8_t>((payload.size() >> 8) & 0xFF));
    body.insert(body.end(), payload.begin(), payload.end());

    uint8_t ck_a, ck_b;
    ubx_checksum(body.data(), body.size(), ck_a, ck_b);

    std::vector<uint8_t> f;
    f.reserve(body.size() + 4);
    f.push_back(UBX_SYNC1);
    f.push_back(UBX_SYNC2);
    f.insert(f.end(), body.begin(), body.end());
    f.push_back(ck_a);
    f.push_back(ck_b);
    return f;
}

// --------------------------- CFG-VALSET (0x06 0x8A) --------------------------
// Value length is encoded in bits [30:28] of the configuration key:
//   0x1 -> 1 byte (bool)  0x2 -> 1 byte  0x3 -> 2 bytes  0x4 -> 4 bytes  0x5 -> 8 bytes
size_t cfg_value_len(uint32_t key) {
    switch ((key >> 28) & 0x7) {
        case 0x1: return 1;
        case 0x2: return 1;
        case 0x3: return 2;
        case 0x4: return 4;
        case 0x5: return 8;
        default:  return 0;
    }
}

struct CfgItem { uint32_t key; uint64_t val; };

// layers bitfield: 0x01=RAM, 0x02=BBR, 0x04=Flash. RAM keeps it non-destructive.
std::vector<uint8_t> cfg_valset(const std::vector<CfgItem>& items, uint8_t layers) {
    std::vector<uint8_t> p;
    p.push_back(0x00);     // version
    p.push_back(layers);   // layers
    p.push_back(0x00);     // reserved
    p.push_back(0x00);
    for (const auto& it : items) {
        for (int i = 0; i < 4; ++i)                       // key, little-endian
            p.push_back(static_cast<uint8_t>((it.key >> (8 * i)) & 0xFF));
        size_t vl = cfg_value_len(it.key);
        for (size_t i = 0; i < vl; ++i)                   // value, little-endian
            p.push_back(static_cast<uint8_t>((it.val >> (8 * i)) & 0xFF));
    }
    return ubx_frame(0x06, 0x8A, p);
}

// ----------------------- Configuration key constants -------------------------
// Verified against u-blox F9 config DB (per-port key sets).
// USB:
constexpr uint32_t K_RAWX_USB   = 0x209102A7; // CFG-MSGOUT-UBX_RXM_RAWX_USB
constexpr uint32_t K_SFRBX_USB  = 0x20910234; // CFG-MSGOUT-UBX_RXM_SFRBX_USB
// UART1:
constexpr uint32_t K_RAWX_UART1 = 0x209102A5; // CFG-MSGOUT-UBX_RXM_RAWX_UART1
constexpr uint32_t K_SFRBX_UART1= 0x20910232; // CFG-MSGOUT-UBX_RXM_SFRBX_UART1
// Measurement / navigation rate:
constexpr uint32_t K_RATE_MEAS  = 0x30210001; // CFG-RATE-MEAS (U2, ms)
constexpr uint32_t K_RATE_NAV   = 0x30210002; // CFG-RATE-NAV  (U2, cycles)
// NMEA output protocol, one key per port (CFG-*OUTPROT-NMEA, L):
constexpr uint32_t K_NMEA_I2C   = 0x10720002; // CFG-I2COUTPROT-NMEA
constexpr uint32_t K_NMEA_UART1 = 0x10740002; // CFG-UART1OUTPROT-NMEA
constexpr uint32_t K_NMEA_UART2 = 0x10760002; // CFG-UART2OUTPROT-NMEA
constexpr uint32_t K_NMEA_USB   = 0x10780002; // CFG-USBOUTPROT-NMEA
constexpr uint32_t K_NMEA_SPI   = 0x107A0002; // CFG-SPIOUTPROT-NMEA

// ------------------------------ serial port ----------------------------------
int open_serial(const std::string& dev, int baud) {
    int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror(("open " + dev).c_str()); return -1; }

    termios tio{};
    if (tcgetattr(fd, &tio) != 0) { perror("tcgetattr"); ::close(fd); return -1; }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1;   // 0.1 s read timeout

    speed_t s;
    switch (baud) {
        case 9600:   s = B9600;   break;
        case 38400:  s = B38400;  break;
        case 115200: s = B115200; break;
        case 230400: s = B230400; break;
        case 460800: s = B460800; break;
        case 921600: s = B921600; break;
        default:     s = B115200; break;
    }
    cfsetispeed(&tio, s);
    cfsetospeed(&tio, s);
    if (tcsetattr(fd, TCSANOW, &tio) != 0) { perror("tcsetattr"); ::close(fd); return -1; }

    // clear O_NONBLOCK so subsequent reads honor VMIN/VTIME
    int fl = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, fl & ~O_NONBLOCK);
    tcflush(fd, TCIOFLUSH);
    return fd;
}

bool write_all(int fd, const uint8_t* buf, size_t n) {
    size_t off = 0;
    while (off < n) {
        ssize_t w = ::write(fd, buf + off, n - off);
        if (w < 0) { if (errno == EINTR) continue; perror("write"); return false; }
        off += static_cast<size_t>(w);
    }
    return true;
}

// ------------------------ lightweight UBX scanner ----------------------------
// Validates frames on the incoming stream so we can report live RAWX/SFRBX
// counts (confirmation the receiver is actually producing raw data).
struct UbxScanner {
    enum State { SYNC1, SYNC2, HEAD, PAY, CKA, CKB } st = SYNC1;
    uint8_t cls = 0, id = 0;
    uint16_t len = 0, got = 0;
    std::vector<uint8_t> head;   // cls,id,len_lo,len_hi + payload for checksum
    uint8_t ck_a = 0, ck_b = 0;

    uint64_t rawx = 0, sfrbx = 0, bad = 0;

    void feed(uint8_t b) {
        switch (st) {
        case SYNC1: if (b == UBX_SYNC1) st = SYNC2; break;
        case SYNC2: st = (b == UBX_SYNC2) ? HEAD : SYNC1;
                    head.clear(); got = 0; break;
        case HEAD:
            head.push_back(b);
            if (head.size() == 4) {
                cls = head[0]; id = head[1];
                len = static_cast<uint16_t>(head[2] | (head[3] << 8));
                st  = (len == 0) ? CKA : PAY;
            }
            break;
        case PAY:
            head.push_back(b);
            if (++got == len) st = CKA;
            break;
        case CKA: ck_a = b; st = CKB; break;
        case CKB: {
            ck_b = b;
            uint8_t a, c; ubx_checksum(head.data(), head.size(), a, c);
            if (a == ck_a && c == ck_b) {
                if (cls == 0x02 && id == 0x15) ++rawx;   // RXM-RAWX
                else if (cls == 0x02 && id == 0x13) ++sfrbx; // RXM-SFRBX
            } else {
                ++bad;
            }
            st = SYNC1;
            break;
        }
        }
    }
};

// ------------------------------ CLI ------------------------------------------
struct Args {
    std::string port = "/dev/ttyACM0";
    std::string out  = "rover.ubx";
    int baud = 115200;
    int rate_hz = 5;             // measurement rate (Hz)
    int duration_s = 0;          // 0 = until Ctrl+C
    bool uart = false;           // target UART1 keys instead of USB
    bool no_config = false;      // skip CFG-VALSET (receiver preconfigured)
};

void usage(const char* p) {
    fprintf(stderr,
        "Usage: %s [--port DEV] [--out FILE.ubx] [--baud N] [--rate HZ]\n"
        "          [--duration S] [--port-target usb|uart1] [--no-config]\n"
        "Defaults: --port /dev/ttyACM0 --out rover.ubx --baud 115200 --rate 5 (USB)\n", p);
}

} // namespace

int main(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&](const char* n)->std::string {
            if (i + 1 >= argc) { fprintf(stderr, "missing value for %s\n", n); exit(2); }
            return argv[++i];
        };
        if      (k == "--port")        a.port = next("--port");
        else if (k == "--out")         a.out  = next("--out");
        else if (k == "--baud")        a.baud = std::stoi(next("--baud"));
        else if (k == "--rate")        a.rate_hz = std::stoi(next("--rate"));
        else if (k == "--duration")    a.duration_s = std::stoi(next("--duration"));
        else if (k == "--port-target") a.uart = (next("--port-target") == "uart1");
        else if (k == "--no-config")   a.no_config = true;
        else if (k == "-h" || k == "--help") { usage(argv[0]); return 0; }
        else { fprintf(stderr, "unknown arg: %s\n", k.c_str()); usage(argv[0]); return 2; }
    }
    if (a.rate_hz < 1) a.rate_hz = 1;

    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);

    int fd = open_serial(a.port, a.baud);
    if (fd < 0) return 1;

    FILE* fout = std::fopen(a.out.c_str(), "wb");
    if (!fout) { perror(("fopen " + a.out).c_str()); ::close(fd); return 1; }

    // ---- configure the receiver (RAM layer) --------------------------------
    if (!a.no_config) {
        uint32_t k_rawx  = a.uart ? K_RAWX_UART1  : K_RAWX_USB;
        uint32_t k_sfrbx = a.uart ? K_SFRBX_UART1 : K_SFRBX_USB;
        uint16_t meas_ms = static_cast<uint16_t>(1000 / a.rate_hz);
        std::vector<CfgItem> items = {
            { k_rawx,      1 },        // enable RAWX on target port
            { k_sfrbx,     1 },        // enable SFRBX on target port
            { K_RATE_MEAS, meas_ms },  // ms between measurements
            { K_RATE_NAV,  1 },        // 1 measurement per nav epoch
            // NMEA off on *every* port. This is not tidiness, and disabling it
            // only on the logged port is not enough.
            //
            // The F9P ships with NMEA on across all ports, and CFG-RATE-MEAS is
            // global -- so asking for 5 Hz raw data also asks for 5 Hz NMEA on
            // ports nobody is reading. The default NMEA set at 5 Hz is ~3.8 kB/s,
            // and UART1's factory-default 38400 baud carries 3.84 kB/s: it backs
            // up permanently, and the transmit buffers it holds come out of a
            // pool shared with USB. The receiver then fails to allocate for the
            // tail of each epoch's output and says so ("txbuf alloc", 1 Hz,
            // forever). RXM-RAWX is last in the burst and the largest message at
            // ~2 kB/epoch, so it is dropped every single time.
            //
            // Measured on a 76-minute F9P session logged off /dev/ttyACM0: NMEA
            // was 79% of the bytes, $GPGSV (early in the burst) arrived 4.29x per
            // epoch while $GLGSV (later) arrived 0.02x, and RAWX stopped dead
            // after 2.8 s -- 14 usable epochs out of ~22800. The whole drive was
            // unpostprocessable and nothing in the stream looked broken.
            { K_NMEA_I2C,   0 },
            { K_NMEA_UART1, 0 },
            { K_NMEA_UART2, 0 },
            { K_NMEA_USB,   0 },
            { K_NMEA_SPI,   0 },
        };
        auto frame = cfg_valset(items, /*layers=*/0x01 /* RAM */);
        if (!write_all(fd, frame.data(), frame.size())) { std::fclose(fout); ::close(fd); return 1; }
        fprintf(stderr, "[cfg] enabled RAWX+SFRBX on %s @ %d Hz, NMEA off (all ports)\n",
                a.uart ? "UART1" : "USB", a.rate_hz);
        usleep(200000);
        tcflush(fd, TCIFLUSH);   // drop the ACK/echo so the log starts clean
    }

    fprintf(stderr, "[log] %s -> %s   (Ctrl+C to stop)\n", a.port.c_str(), a.out.c_str());

    UbxScanner scan;
    uint64_t total_bytes = 0;
    uint8_t buf[8192];
    auto t0   = std::chrono::steady_clock::now();
    auto tlog = t0;

    // RAWX stalling while the rest of the stream keeps flowing is the failure
    // this counter exists to catch. It looks like nothing is wrong -- bytes
    // arrive, SFRBX climbs, the receiver holds a fix -- and the log is worthless.
    uint64_t last_rawx = 0, last_bytes = 0;
    auto     t_rawx_change = t0;
    bool     stall_warned = false, ever_stalled = false;

    while (g_run.load()) {
        ssize_t r = ::read(fd, buf, sizeof(buf));
        if (r < 0) { if (errno == EINTR) continue; perror("read"); break; }
        if (r > 0) {
            if (std::fwrite(buf, 1, static_cast<size_t>(r), fout) != static_cast<size_t>(r)) {
                perror("fwrite"); break;
            }
            total_bytes += static_cast<uint64_t>(r);
            for (ssize_t i = 0; i < r; ++i) scan.feed(buf[i]);
        }

        auto now = std::chrono::steady_clock::now();
        if (now - tlog >= std::chrono::seconds(1)) {
            double secs = std::chrono::duration<double>(now - t0).count();
            if (scan.rawx != last_rawx) {
                last_rawx = scan.rawx; t_rawx_change = now; stall_warned = false;
            }
            double stalled = std::chrono::duration<double>(now - t_rawx_change).count();
            fprintf(stderr, "\r[%.0fs] RAWX(epochs)=%llu  SFRBX=%llu  bad=%llu  %.1f kB   ",
                    secs, (unsigned long long)scan.rawx, (unsigned long long)scan.sfrbx,
                    (unsigned long long)scan.bad, total_bytes / 1024.0);
            std::fflush(stderr);

            // Bytes still arriving but RAWX frozen => the receiver is dropping it.
            if (!stall_warned && scan.rawx > 0 && stalled >= 5.0 && total_bytes > last_bytes) {
                fprintf(stderr,
                    "\n[warn] RAWX stalled at %llu epochs for %.0f s while %.1f kB/s still\n"
                    "       arrives -- the receiver is dropping RXM-RAWX. Without it this\n"
                    "       log cannot be post-processed at all. Stop and fix it now:\n"
                    "       lower --rate, or check nothing re-enabled NMEA / another port\n"
                    "       is backed up and holding the transmit buffer pool.\n",
                    (unsigned long long)scan.rawx, stalled,
                    (total_bytes - last_bytes) / 1024.0);
                stall_warned = true; ever_stalled = true;
            }
            tlog = now;
            last_bytes = total_bytes;
        }
        if (a.duration_s > 0 &&
            std::chrono::duration<double>(now - t0).count() >= a.duration_s) break;
    }

    std::fflush(fout);
    std::fclose(fout);
    ::close(fd);

    fprintf(stderr,
        "\n[done] wrote %s : %.1f kB, RAWX epochs=%llu, SFRBX=%llu, bad frames=%llu\n",
        a.out.c_str(), total_bytes / 1024.0,
        (unsigned long long)scan.rawx, (unsigned long long)scan.sfrbx,
        (unsigned long long)scan.bad);
    // "Some RAWX" is not the same as "enough RAWX": the session this check was
    // added for ended with 14 epochs of an expected ~22800 and looked healthy.
    double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    unsigned long long expect = (unsigned long long)(secs * a.rate_hz);
    if (scan.rawx == 0) {
        fprintf(stderr,
            "[warn] no RAWX epochs seen. Check: correct --port-target (usb vs uart1),\n"
            "       firmware is HPG (High Precision), and antenna has sky view.\n");
    } else if (expect > 0 && scan.rawx * 10 < expect * 9) {
        fprintf(stderr,
            "[warn] RAWX epochs %llu of ~%llu expected at %d Hz -- %.0f%% MISSING.\n"
            "       The receiver dropped them; this log will post-process to a\n"
            "       fraction of the drive%s. Do not rely on this data.\n",
            (unsigned long long)scan.rawx, expect, a.rate_hz,
            100.0 * (1.0 - (double)scan.rawx / (double)expect),
            ever_stalled ? " (RAWX stalled during the run)" : "");
    }
    return 0;
}
