// main.cpp - Win32 dialog front-end for the NMEA / AIS test data generator.
#include <windows.h>
#include <commctrl.h>
#include <richedit.h>
#include <string>
#include <deque>
#include <set>
#include <mutex>
#include <memory>
#include <utility>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>

#include "resource.h"
#include "Network.h"
#include "Simulation.h"
#include "Nmea.h"
#include "N2k.h"
#include "ApInput.h"

#pragma comment(lib, "Comctl32.lib")

using namespace nmea;

namespace {

constexpr int kTimerId = 1;
constexpr size_t kMaxLogChars = 60000; // trim the edit control past this

// Log line colour categories.
enum class LogKind { Generated = 0, Incoming = 1, Note = 2, Decoded = 3, Error = 4 };

struct AppState {
    std::unique_ptr<NetworkServer> net;
    std::unique_ptr<Simulation> sim;

    std::mutex logMutex;
    std::deque<std::pair<std::string, LogKind>> pending; // lines awaiting display
    size_t logLen = 0;               // approx chars currently in the edit box
    bool running = false;
    bool apEngagedPrev = false;      // last seen autopilot state (for notices)
    std::string baseStatus;          // status text without the autopilot suffix
};

AppState g_app;

// --- small control helpers -------------------------------------------------

std::wstring Widen(const std::string& s) {
    if (s.empty()) return L"";
    int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), (int)s.size(), nullptr, 0);
    std::wstring w(n, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), (int)s.size(), &w[0], n);
    return w;
}

std::string Narrow(const std::wstring& w) {
    if (w.empty()) return "";
    int n = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), (int)w.size(), nullptr, 0, nullptr, nullptr);
    std::string s(n, '\0');
    WideCharToMultiByte(CP_UTF8, 0, w.c_str(), (int)w.size(), &s[0], n, nullptr, nullptr);
    return s;
}

std::string GetText(HWND dlg, int id) {
    wchar_t buf[256];
    GetDlgItemTextW(dlg, id, buf, 256);
    return Narrow(buf);
}

void SetText(HWND dlg, int id, const std::string& s) {
    SetDlgItemTextW(dlg, id, Widen(s).c_str());
}

double GetDouble(HWND dlg, int id, double def) {
    try { return std::stod(GetText(dlg, id)); } catch (...) { return def; }
}

int GetIntField(HWND dlg, int id, int def) {
    try { return std::stoi(GetText(dlg, id)); } catch (...) { return def; }
}

bool IsChecked(HWND dlg, int id) {
    return SendDlgItemMessageW(dlg, id, BM_GETCHECK, 0, 0) == BST_CHECKED;
}

void AddComboItem(HWND dlg, int id, const wchar_t* text) {
    SendDlgItemMessageW(dlg, id, CB_ADDSTRING, 0, (LPARAM)text);
}

int ComboSel(HWND dlg, int id, int def) {
    LRESULT r = SendDlgItemMessageW(dlg, id, CB_GETCURSEL, 0, 0);
    return (r == CB_ERR) ? def : static_cast<int>(r);
}

void SetComboSel(HWND dlg, int id, int sel) {
    int count = static_cast<int>(SendDlgItemMessageW(dlg, id, CB_GETCOUNT, 0, 0));
    if (sel < 0 || sel >= count) sel = 0;
    SendDlgItemMessageW(dlg, id, CB_SETCURSEL, sel, 0);
}

int TargetId(int i, int field);

// --- persistent settings ---------------------------------------------------

std::string SettingsPath() {
    char appdata[MAX_PATH]{};
    DWORD n = GetEnvironmentVariableA("APPDATA", appdata, MAX_PATH);
    std::string dir = (n > 0 && n < MAX_PATH)
        ? std::string(appdata) + "\\NMEA-Generator"
        : ".";
    CreateDirectoryA(dir.c_str(), nullptr);
    return dir + "\\settings.ini";
}

std::map<std::string, std::string> LoadSettingsFile() {
    std::map<std::string, std::string> s;
    std::ifstream in(SettingsPath());
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        s[line.substr(0, eq)] = line.substr(eq + 1);
    }
    return s;
}

std::string Setting(const std::map<std::string, std::string>& s,
                    const std::string& key, const std::string& def) {
    auto it = s.find(key);
    return it == s.end() ? def : it->second;
}

int SettingInt(const std::map<std::string, std::string>& s,
               const std::string& key, int def) {
    try { return std::stoi(Setting(s, key, "")); } catch (...) { return def; }
}

void SaveSettings(HWND dlg) {
    std::ofstream out(SettingsPath(), std::ios::trunc);
    if (!out) return;

    out << "# NMEA Generator settings\n";
    out << "tcp_port=" << GetText(dlg, IDC_TCP_PORT) << "\n";
    out << "udp_port=" << GetText(dlg, IDC_UDP_PORT) << "\n";
    out << "protocol=" << ComboSel(dlg, IDC_PROTOCOL, 0) << "\n";

    out << "own_lat=" << GetText(dlg, IDC_OWN_LAT) << "\n";
    out << "own_lon=" << GetText(dlg, IDC_OWN_LON) << "\n";
    out << "own_width=" << GetText(dlg, IDC_OWN_WIDTH) << "\n";
    out << "own_height=" << GetText(dlg, IDC_OWN_HEIGHT) << "\n";
    out << "own_shape=" << ComboSel(dlg, IDC_OWN_SHAPE, 0) << "\n";
    out << "own_speed=" << GetText(dlg, IDC_OWN_SPEED) << "\n";

    for (int i = 0; i < kNumTargets; ++i) {
        std::string p = "target" + std::to_string(i + 1) + "_";
        out << p << "enabled=" << (IsChecked(dlg, TargetId(i, 0)) ? 1 : 0) << "\n";
        out << p << "class=" << ComboSel(dlg, TargetId(i, 1), 0) << "\n";
        out << p << "shape=" << ComboSel(dlg, TargetId(i, 2), 0) << "\n";
        out << p << "offx=" << GetText(dlg, TargetId(i, 3)) << "\n";
        out << p << "offy=" << GetText(dlg, TargetId(i, 4)) << "\n";
        out << p << "speed=" << GetText(dlg, TargetId(i, 5)) << "\n";
    }
}

// --- defaults & combos -----------------------------------------------------

void PopulateShapeCombo(HWND dlg, int id, int sel) {
    AddComboItem(dlg, id, L"Circle");
    AddComboItem(dlg, id, L"Square");
    AddComboItem(dlg, id, L"Figure-8");
    SendDlgItemMessageW(dlg, id, CB_SETCURSEL, sel, 0);
}

void PopulateClassCombo(HWND dlg, int id, int sel) {
    AddComboItem(dlg, id, L"A");
    AddComboItem(dlg, id, L"B");
    AddComboItem(dlg, id, L"SAR-FW");
    AddComboItem(dlg, id, L"SAR-H");
    SendDlgItemMessageW(dlg, id, CB_SETCURSEL, sel, 0);
}

void PopulateProtocolCombo(HWND dlg) {
    AddComboItem(dlg, IDC_PROTOCOL, L"NMEA 0183");
    AddComboItem(dlg, IDC_PROTOCOL, L"N2K ASCII");
    SendDlgItemMessageW(dlg, IDC_PROTOCOL, CB_SETCURSEL, 0, 0);
}

int TargetId(int i, int field) { return 1100 + i * 10 + field; }

void InitControls(HWND dlg) {
    const auto settings = LoadSettingsFile();

    SetText(dlg, IDC_TCP_PORT, "10110");
    SetText(dlg, IDC_UDP_PORT, "10110");
    PopulateProtocolCombo(dlg);

    SetText(dlg, IDC_OWN_LAT, "50.800000");
    SetText(dlg, IDC_OWN_LON, "-1.100000");
    SetText(dlg, IDC_OWN_WIDTH, "5.0");
    SetText(dlg, IDC_OWN_HEIGHT, "5.0");
    PopulateShapeCombo(dlg, IDC_OWN_SHAPE, 0);
    SetText(dlg, IDC_OWN_SPEED, "10.0");

    const double defOffX[4] = { -2.0, -0.5, 1.0, 2.5 };
    const double defOffY[4] = { 1.0, -1.0, 1.0, -1.0 };
    for (int i = 0; i < kNumTargets; ++i) {
        CheckDlgButton(dlg, TargetId(i, 0), BST_CHECKED);
        PopulateClassCombo(dlg, TargetId(i, 1), (i % 2 == 0) ? 0 : 1);
        PopulateShapeCombo(dlg, TargetId(i, 2), i % 3);
        char b[32];
        std::snprintf(b, sizeof(b), "%.1f", defOffX[i]); SetText(dlg, TargetId(i, 3), b);
        std::snprintf(b, sizeof(b), "%.1f", defOffY[i]); SetText(dlg, TargetId(i, 4), b);
        SetText(dlg, TargetId(i, 5), "8.0");
    }

    SetText(dlg, IDC_TCP_PORT, Setting(settings, "tcp_port", GetText(dlg, IDC_TCP_PORT)));
    SetText(dlg, IDC_UDP_PORT, Setting(settings, "udp_port", GetText(dlg, IDC_UDP_PORT)));
    SetComboSel(dlg, IDC_PROTOCOL, SettingInt(settings, "protocol", 0));

    SetText(dlg, IDC_OWN_LAT, Setting(settings, "own_lat", GetText(dlg, IDC_OWN_LAT)));
    SetText(dlg, IDC_OWN_LON, Setting(settings, "own_lon", GetText(dlg, IDC_OWN_LON)));
    SetText(dlg, IDC_OWN_WIDTH, Setting(settings, "own_width", GetText(dlg, IDC_OWN_WIDTH)));
    SetText(dlg, IDC_OWN_HEIGHT, Setting(settings, "own_height", GetText(dlg, IDC_OWN_HEIGHT)));
    SetComboSel(dlg, IDC_OWN_SHAPE, SettingInt(settings, "own_shape", 0));
    SetText(dlg, IDC_OWN_SPEED, Setting(settings, "own_speed", GetText(dlg, IDC_OWN_SPEED)));

    for (int i = 0; i < kNumTargets; ++i) {
        std::string p = "target" + std::to_string(i + 1) + "_";
        CheckDlgButton(dlg, TargetId(i, 0),
                       SettingInt(settings, p + "enabled", IsChecked(dlg, TargetId(i, 0)) ? 1 : 0)
                           ? BST_CHECKED : BST_UNCHECKED);
        SetComboSel(dlg, TargetId(i, 1), SettingInt(settings, p + "class", ComboSel(dlg, TargetId(i, 1), 0)));
        SetComboSel(dlg, TargetId(i, 2), SettingInt(settings, p + "shape", ComboSel(dlg, TargetId(i, 2), 0)));
        SetText(dlg, TargetId(i, 3), Setting(settings, p + "offx", GetText(dlg, TargetId(i, 3))));
        SetText(dlg, TargetId(i, 4), Setting(settings, p + "offy", GetText(dlg, TargetId(i, 4))));
        SetText(dlg, TargetId(i, 5), Setting(settings, p + "speed", GetText(dlg, TargetId(i, 5))));
    }
}

SimConfig ReadConfig(HWND dlg) {
    SimConfig cfg;
    cfg.protocol = (SendDlgItemMessageW(dlg, IDC_PROTOCOL, CB_GETCURSEL, 0, 0) == 1)
        ? ProtocolMode::Nmea2000
        : ProtocolMode::Nmea0183;
    cfg.ownship.centreLat = GetDouble(dlg, IDC_OWN_LAT, 50.0);
    cfg.ownship.centreLon = GetDouble(dlg, IDC_OWN_LON, -1.0);
    cfg.ownship.widthNm = GetDouble(dlg, IDC_OWN_WIDTH, 5.0);
    cfg.ownship.heightNm = GetDouble(dlg, IDC_OWN_HEIGHT, 5.0);
    cfg.ownship.shape = (Shape)SendDlgItemMessageW(dlg, IDC_OWN_SHAPE, CB_GETCURSEL, 0, 0);
    cfg.ownship.speed = GetDouble(dlg, IDC_OWN_SPEED, 10.0);

    for (int i = 0; i < kNumTargets; ++i) {
        TargetConfig& t = cfg.targets[i];
        t.enabled = IsChecked(dlg, TargetId(i, 0));
        int kind = ComboSel(dlg, TargetId(i, 1), 0);
        if (kind < 0 || kind > 3) kind = 0;
        t.kind = static_cast<AisTargetKind>(kind);
        t.classA = (t.kind == AisTargetKind::ClassA);
        t.shape = (Shape)SendDlgItemMessageW(dlg, TargetId(i, 2), CB_GETCURSEL, 0, 0);
        t.offsetX = GetDouble(dlg, TargetId(i, 3), 0.0);
        t.offsetY = GetDouble(dlg, TargetId(i, 4), 0.0);
        t.speed = GetDouble(dlg, TargetId(i, 5), 8.0);
    }
    return cfg;
}

// --- log handling ----------------------------------------------------------

void QueueLine(const std::string& line, LogKind kind) {
    std::lock_guard<std::mutex> lock(g_app.logMutex);
    g_app.pending.emplace_back(line, kind);
    if (g_app.pending.size() > 2000) g_app.pending.pop_front();
}

COLORREF ColorFor(LogKind kind) {
    switch (kind) {
        case LogKind::Incoming: return RGB(0, 0, 210);    // blue
        case LogKind::Note:     return RGB(150, 0, 150);  // magenta
        case LogKind::Decoded:  return RGB(0, 130, 0);    // green
        case LogKind::Error:    return RGB(200, 0, 0);    // red
        case LogKind::Generated:
        default:                return RGB(0, 0, 0);      // black
    }
}

// Append text to the RichEdit log in the given colour, scrolling to the end.
void AppendColored(HWND edit, const std::wstring& text, COLORREF color) {
    CHARFORMAT2W cf{};
    cf.cbSize = sizeof(cf);
    cf.dwMask = CFM_COLOR;
    cf.crTextColor = color;

    LRESULT len = SendMessageW(edit, WM_GETTEXTLENGTH, 0, 0);
    SendMessageW(edit, EM_SETSEL, len, len);
    SendMessageW(edit, EM_SETCHARFORMAT, SCF_SELECTION, (LPARAM)&cf);
    SendMessageW(edit, EM_REPLACESEL, FALSE, (LPARAM)text.c_str());
    SendMessageW(edit, WM_VSCROLL, SB_BOTTOM, 0);
}

void FlushLog(HWND dlg) {
    std::deque<std::pair<std::string, LogKind>> lines;
    {
        std::lock_guard<std::mutex> lock(g_app.logMutex);
        if (g_app.pending.empty()) return;
        lines.swap(g_app.pending);
    }

    HWND edit = GetDlgItem(dlg, IDC_LOG);

    size_t incoming = 0;
    for (auto& l : lines) incoming += l.first.size();

    // Trim by clearing if the box is getting large.
    g_app.logLen += incoming;
    if (g_app.logLen > kMaxLogChars) {
        SetWindowTextW(edit, L"");
        g_app.logLen = incoming;
    }

    // Coalesce consecutive same-colour lines into one append.
    size_t i = 0;
    while (i < lines.size()) {
        LogKind kind = lines[i].second;
        std::string chunk;
        while (i < lines.size() && lines[i].second == kind) {
            chunk += lines[i].first; // already includes a trailing newline
            ++i;
        }
        AppendColored(edit, Widen(chunk), ColorFor(kind));
    }
}

void QueueNote(const std::string& text) {
    QueueLine("// " + text + "\r\n", LogKind::Note);
}

// --- start / stop ----------------------------------------------------------

void SetEditingEnabled(HWND dlg, bool enabled) {
    const int ids[] = { IDC_TCP_PORT, IDC_UDP_PORT, IDC_PROTOCOL, IDC_OWN_LAT, IDC_OWN_LON,
                        IDC_OWN_WIDTH, IDC_OWN_HEIGHT, IDC_OWN_SHAPE, IDC_OWN_SPEED };
    for (int id : ids) EnableWindow(GetDlgItem(dlg, id), enabled);
    for (int i = 0; i < kNumTargets; ++i)
        for (int f = 0; f < 6; ++f)
            EnableWindow(GetDlgItem(dlg, TargetId(i, f)), enabled);
}

void StartSimulation(HWND dlg) {
    unsigned short tcp = (unsigned short)GetIntField(dlg, IDC_TCP_PORT, 10110);
    unsigned short udp = (unsigned short)GetIntField(dlg, IDC_UDP_PORT, 10110);
    SimConfig cfg = ReadConfig(dlg);
    SaveSettings(dlg);

    std::string err;
    if (!g_app.net->Start(tcp, udp, err)) {
        MessageBoxW(dlg, Widen("Failed to start network: " + err).c_str(),
                    L"NMEA Generator", MB_ICONERROR | MB_OK);
        return;
    }

    g_app.sim->SetConfig(cfg);
    g_app.sim->Start();
    g_app.running = true;
    g_app.apEngagedPrev = false;

    SetText(dlg, IDC_START, "Stop Simulation");
    EnableWindow(GetDlgItem(dlg, IDC_MOB), TRUE);
    char status[160];
    const char* protocol = (cfg.protocol == ProtocolMode::Nmea2000)
        ? "NMEA 2000 Actisense ASCII"
        : "NMEA 0183";
    std::snprintf(status, sizeof(status),
                  "Running %s - TCP listening on %u, UDP on %u.",
                  protocol, tcp, udp);
    g_app.baseStatus = status;
    SetText(dlg, IDC_STATUS, g_app.baseStatus);
    SetEditingEnabled(dlg, false);
}

void StopSimulation(HWND dlg) {
    SaveSettings(dlg);
    g_app.sim->Stop();
    g_app.net->Stop();
    g_app.running = false;

    SetText(dlg, IDC_START, "Start Simulation");
    EnableWindow(GetDlgItem(dlg, IDC_MOB), FALSE);
    SetText(dlg, IDC_STATUS, "Stopped.");
    SetEditingEnabled(dlg, true);
}

// --- dialog procedure ------------------------------------------------------

INT_PTR CALLBACK DlgProc(HWND dlg, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_INITDIALOG: {
        g_app.net = std::make_unique<NetworkServer>();
        g_app.sim = std::make_unique<Simulation>([](const std::string& line) {
            g_app.net->Send(line);
            QueueLine(line, LogKind::Generated);
        });
        // Inbound sentences: ignore echoes of our own output, log the rest in
        // the incoming colour, validate/decode autopilot sentences, and feed
        // them to the autopilot.
        g_app.net->SetReceiveSink([](const std::string& line) {
            static const std::set<std::string> generated =
                { "GGA", "RMC", "VTG", "GLL", "VHW", "MWV", "MWD", "VDM", "VDO" };
            if (!line.empty() && line[0] == 'A') {
                nmea::N2kMessage msg;
                std::string decodeError;
                if (nmea::DecodeActisenseAscii(line, msg, decodeError) &&
                    decodeError.empty() && nmea::IsGeneratedN2kPgn(msg.pgn)) {
                    return;
                }
            } else if (generated.count(nmea::SentenceFormatter(line))) {
                return;
            }
            QueueLine(line + "\r\n", LogKind::Incoming);

            // Validate and decode APB/RMB/XTE or NMEA 2000 autopilot PGNs.
            nmea::ApInput in;
            if (nmea::ParseAutopilotInput(line, in) ||
                nmea::ParseN2kAutopilotInput(line, in)) {
                if (in.ok())
                    QueueLine("    " + in.formatter + " OK   " + in.DecodedSummary() + "\r\n",
                              LogKind::Decoded);
                else
                    QueueLine("    " + in.formatter + " ERROR: " + in.ErrorText() + "\r\n",
                              LogKind::Error);
            }

            if (g_app.sim) g_app.sim->OnIncomingSentence(line);
        });
        InitControls(dlg);
        EnableWindow(GetDlgItem(dlg, IDC_MOB), FALSE);
        SetTimer(dlg, kTimerId, 200, nullptr);
        return TRUE;
    }

    case WM_TIMER:
        if (wp == kTimerId) {
            FlushLog(dlg);
            if (g_app.running && g_app.sim) {
                bool engaged = g_app.sim->AutopilotEngaged();
                if (engaged != g_app.apEngagedPrev) {
                    QueueNote(engaged
                        ? "Autopilot engaged - steering from received autopilot data."
                        : "Autopilot signal lost - resuming predefined pattern.");
                    SetText(dlg, IDC_STATUS, g_app.baseStatus +
                        (engaged ? "  [AUTOPILOT ENGAGED]" : ""));
                    g_app.apEngagedPrev = engaged;
                }
            }
        }
        return TRUE;

    case WM_COMMAND:
        switch (LOWORD(wp)) {
        case IDC_START:
            if (g_app.running) StopSimulation(dlg);
            else StartSimulation(dlg);
            return TRUE;
        case IDC_CLEAR_LOG:
            SetWindowTextW(GetDlgItem(dlg, IDC_LOG), L"");
            g_app.logLen = 0;
            return TRUE;
        case IDC_MOB:
            if (g_app.running && g_app.sim) {
                g_app.sim->TriggerMob();
                QueueNote("AIS MOB burst started for 2 minutes.");
            }
            return TRUE;
        case IDCANCEL:
            SaveSettings(dlg);
            DestroyWindow(dlg);
            return TRUE;
        }
        // Apply live config edits to a running simulation.
        if (g_app.running && HIWORD(wp) == CBN_SELCHANGE)
            g_app.sim->SetConfig(ReadConfig(dlg));
        return FALSE;

    case WM_DESTROY:
        KillTimer(dlg, kTimerId);
        if (g_app.sim) g_app.sim->Stop();
        if (g_app.net) g_app.net->Stop();
        PostQuitMessage(0);
        return TRUE;

    case WM_CLOSE:
        SaveSettings(dlg);
        DestroyWindow(dlg);
        return TRUE;
    }
    return FALSE;
}

} // namespace

int WINAPI wWinMain(HINSTANCE hInst, HINSTANCE, LPWSTR, int) {
    INITCOMMONCONTROLSEX icc{ sizeof(icc), ICC_STANDARD_CLASSES };
    InitCommonControlsEx(&icc);

    // RICHEDIT50W (used by the log control) lives in Msftedit.dll.
    HMODULE richedit = LoadLibraryW(L"Msftedit.dll");

    HWND dlg = CreateDialogParamW(hInst, MAKEINTRESOURCEW(IDD_MAIN), nullptr, DlgProc, 0);
    if (!dlg) { if (richedit) FreeLibrary(richedit); return 1; }
    ShowWindow(dlg, SW_SHOW);

    // Apply non-network config changes live as the user edits text fields.
    MSG msg;
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        if (!IsDialogMessageW(dlg, &msg)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }
    if (richedit) FreeLibrary(richedit);
    return 0;
}
