// main.cpp - Win32 dialog front-end for the NMEA / AIS test data generator.
#include <windows.h>
#include <commctrl.h>
#include <string>
#include <deque>
#include <mutex>
#include <memory>
#include <cstdio>

#include "resource.h"
#include "Network.h"
#include "Simulation.h"

#pragma comment(lib, "Comctl32.lib")

using namespace nmea;

namespace {

constexpr int kTimerId = 1;
constexpr size_t kMaxLogChars = 60000; // trim the edit control past this

struct AppState {
    std::unique_ptr<NetworkServer> net;
    std::unique_ptr<Simulation> sim;

    std::mutex logMutex;
    std::deque<std::string> pending; // lines awaiting display
    size_t logLen = 0;               // approx chars currently in the edit box
    bool running = false;
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
    SendDlgItemMessageW(dlg, id, CB_SETCURSEL, sel, 0);
}

int TargetId(int i, int field) { return 1100 + i * 10 + field; }

void InitControls(HWND dlg) {
    SetText(dlg, IDC_TCP_PORT, "10110");
    SetText(dlg, IDC_UDP_PORT, "10110");

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
}

SimConfig ReadConfig(HWND dlg) {
    SimConfig cfg;
    cfg.ownship.centreLat = GetDouble(dlg, IDC_OWN_LAT, 50.0);
    cfg.ownship.centreLon = GetDouble(dlg, IDC_OWN_LON, -1.0);
    cfg.ownship.widthNm = GetDouble(dlg, IDC_OWN_WIDTH, 5.0);
    cfg.ownship.heightNm = GetDouble(dlg, IDC_OWN_HEIGHT, 5.0);
    cfg.ownship.shape = (Shape)SendDlgItemMessageW(dlg, IDC_OWN_SHAPE, CB_GETCURSEL, 0, 0);
    cfg.ownship.speed = GetDouble(dlg, IDC_OWN_SPEED, 10.0);

    for (int i = 0; i < kNumTargets; ++i) {
        TargetConfig& t = cfg.targets[i];
        t.enabled = IsChecked(dlg, TargetId(i, 0));
        t.classA = (SendDlgItemMessageW(dlg, TargetId(i, 1), CB_GETCURSEL, 0, 0) == 0);
        t.shape = (Shape)SendDlgItemMessageW(dlg, TargetId(i, 2), CB_GETCURSEL, 0, 0);
        t.offsetX = GetDouble(dlg, TargetId(i, 3), 0.0);
        t.offsetY = GetDouble(dlg, TargetId(i, 4), 0.0);
        t.speed = GetDouble(dlg, TargetId(i, 5), 8.0);
    }
    return cfg;
}

// --- log handling ----------------------------------------------------------

void QueueLine(const std::string& line) {
    std::lock_guard<std::mutex> lock(g_app.logMutex);
    g_app.pending.push_back(line);
    if (g_app.pending.size() > 2000) g_app.pending.pop_front();
}

void FlushLog(HWND dlg) {
    std::deque<std::string> lines;
    {
        std::lock_guard<std::mutex> lock(g_app.logMutex);
        if (g_app.pending.empty()) return;
        lines.swap(g_app.pending);
    }

    std::string chunk;
    for (auto& l : lines) chunk += l; // each line already ends with CRLF

    HWND edit = GetDlgItem(dlg, IDC_LOG);
    int len = GetWindowTextLengthW(edit);

    // Trim from the top if the box is getting large.
    g_app.logLen += chunk.size();
    if (g_app.logLen > kMaxLogChars) {
        SetWindowTextW(edit, L"");
        len = 0;
        g_app.logLen = chunk.size();
    }

    SendMessageW(edit, EM_SETSEL, len, len);
    SendMessageW(edit, EM_REPLACESEL, FALSE, (LPARAM)Widen(chunk).c_str());
}

// --- start / stop ----------------------------------------------------------

void SetEditingEnabled(HWND dlg, bool enabled) {
    const int ids[] = { IDC_TCP_PORT, IDC_UDP_PORT, IDC_OWN_LAT, IDC_OWN_LON,
                        IDC_OWN_WIDTH, IDC_OWN_HEIGHT, IDC_OWN_SHAPE, IDC_OWN_SPEED };
    for (int id : ids) EnableWindow(GetDlgItem(dlg, id), enabled);
    for (int i = 0; i < kNumTargets; ++i)
        for (int f = 0; f < 6; ++f)
            EnableWindow(GetDlgItem(dlg, TargetId(i, f)), enabled);
}

void StartSimulation(HWND dlg) {
    unsigned short tcp = (unsigned short)GetIntField(dlg, IDC_TCP_PORT, 10110);
    unsigned short udp = (unsigned short)GetIntField(dlg, IDC_UDP_PORT, 10110);

    std::string err;
    if (!g_app.net->Start(tcp, udp, err)) {
        MessageBoxW(dlg, Widen("Failed to start network: " + err).c_str(),
                    L"NMEA Generator", MB_ICONERROR | MB_OK);
        return;
    }

    g_app.sim->SetConfig(ReadConfig(dlg));
    g_app.sim->Start();
    g_app.running = true;

    SetText(dlg, IDC_START, "Stop Simulation");
    char status[128];
    std::snprintf(status, sizeof(status),
                  "Running - TCP listening on %u, UDP broadcast on %u.", tcp, udp);
    SetText(dlg, IDC_STATUS, status);
    SetEditingEnabled(dlg, false);
}

void StopSimulation(HWND dlg) {
    g_app.sim->Stop();
    g_app.net->Stop();
    g_app.running = false;

    SetText(dlg, IDC_START, "Start Simulation");
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
            QueueLine(line);
        });
        InitControls(dlg);
        SetTimer(dlg, kTimerId, 200, nullptr);
        return TRUE;
    }

    case WM_TIMER:
        if (wp == kTimerId) FlushLog(dlg);
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
        case IDCANCEL:
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
        DestroyWindow(dlg);
        return TRUE;
    }
    return FALSE;
}

} // namespace

int WINAPI wWinMain(HINSTANCE hInst, HINSTANCE, LPWSTR, int) {
    INITCOMMONCONTROLSEX icc{ sizeof(icc), ICC_STANDARD_CLASSES };
    InitCommonControlsEx(&icc);

    HWND dlg = CreateDialogParamW(hInst, MAKEINTRESOURCEW(IDD_MAIN), nullptr, DlgProc, 0);
    if (!dlg) return 1;
    ShowWindow(dlg, SW_SHOW);

    // Apply non-network config changes live as the user edits text fields.
    MSG msg;
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        if (!IsDialogMessageW(dlg, &msg)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }
    return 0;
}
