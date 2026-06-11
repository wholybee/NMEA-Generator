---
name: build-toolchain
description: How to build NMEA-Generator and the VS toolset quirk on this machine
metadata:
  type: project
---

NMEA-Generator is a native Win32 C++ app (no Qt, no external deps; uses Win32 + Winsock `Ws2_32.lib`). Build with the VS2026 MSBuild at `C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe`, e.g. `MSBuild NMEA-Generator.sln /p:Configuration=Release /p:Platform=x64`. Output: `build\x64\Release\NMEA-Generator.exe`.

This machine's VS2026 install only has the **v145** platform toolset (and ClangCL) — **v143 (VS2022) is NOT installed**. So the .vcxproj auto-selects the toolset by `$(VisualStudioVersion)`: v145 when >= 18.0, else v143. Forcing v143 here will fail unless that toolset is added via the VS Installer.

Console tests under `test\` (verify.cpp, netcheck.cpp) compile with `cl` from a `vcvars64.bat` dev shell.
