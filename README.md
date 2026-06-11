# NMEA Generator

A Windows desktop application that generates **NMEA 0183** test data — including
**AIS** (AIVDM) messages — and serves it simultaneously over a **TCP server** and
**UDP broadcast**. Useful for exercising chart plotters, ECDIS, AIS displays, and
other marine navigation software without live sensors.

![dialog](docs/screenshot.png)

## Features

### Ownship (simulated own vessel)
Position is driven around a configurable path (circle / square / figure-eight)
centred on a latitude/longitude you choose, within a simulation box sized in
nautical miles. The following sentences are emitted at ~1 Hz:

| Sentence | Content |
|----------|---------|
| `GGA` | GPS fix (position, time) |
| `RMC` | Position, COG, SOG, date |
| `VTG` | Course & speed over ground |
| `GLL` | Geographic position |
| `VHW` | Speed through water & heading |
| `MWV` (R) | Apparent wind angle & speed |
| `MWV` (T) | True wind (relative to vessel) |
| `MWD` | True wind direction & speed |

COG/SOG are derived from the actual motion along the path; apparent wind is
computed by subtracting vessel motion from a synthesised true wind.

### AIS targets (4)
Each target travels its own path with an optional X/Y offset from the simulation
centre (paths may extend beyond the box when offset). Targets can be Class A or
Class B and toggled on/off individually.

* **Dynamic data — every 6 s:** position, SOG, COG, heading, ROT, UTC second
  * Class A → AIS message **type 1**
  * Class B → AIS message **type 18**
* **Static data — every 60 s:** MMSI, IMO, name, call sign, ship type, dimensions
  * Class A → AIS message **type 5** (two-part)
  * Class B → AIS message **type 24** (Part A + Part B)

MMSI, IMO, name, call sign, ship type and dimensions are pseudo-randomly but
deterministically generated per target. COG, heading and ROT are calculated from
motion.

## Building

Requires Windows 10/11 and Visual Studio with the C++ desktop workload.

### Visual Studio (2026 or 2022)
Open `NMEA-Generator.sln` and build (x64, Debug or Release).

The platform toolset is selected automatically:
* **Visual Studio 2026** → `v145`
* **Visual Studio 2022** → `v143`

Override with `/p:PlatformToolset=...` if needed (e.g. `ClangCL`).

### Command line
```powershell
& "<VS>\MSBuild\Current\Bin\MSBuild.exe" NMEA-Generator.sln /p:Configuration=Release /p:Platform=x64
```
The executable is written to `build\x64\Release\NMEA-Generator.exe`.

There are no external dependencies — the app uses only the Win32 API and Winsock
(`Ws2_32.lib`). Qt is **not** required.

## Using it

1. Set the **TCP port** to listen on and the **UDP port** to broadcast on
   (default `10110`, the common OpenCPN NMEA port).
2. Configure the ownship centre, area size, path shape and speed.
3. Enable/configure the AIS targets.
4. Click **Start Simulation**. Generated sentences appear in the log and are
   streamed to every connected TCP client and broadcast over UDP.

To receive the data, point your software at the machine's IP on the TCP port, or
listen for UDP broadcasts on the UDP port. For a quick check on the same machine:

```powershell
# TCP
$c = New-Object System.Net.Sockets.TcpClient('127.0.0.1', 10110)
$r = New-Object System.IO.StreamReader($c.GetStream())
while ($true) { $r.ReadLine() }
```

## Project layout

```
NMEA-Generator.sln
NMEA-Generator.vcxproj
src/
  Geo.h            Path geometry & lat/lon conversion
  Nmea.h/.cpp      NMEA 0183 sentence building + checksum
  Ais.h/.cpp       AIS 6-bit payload encoding (types 1/5/18/24)
  Network.h/.cpp   TCP server + UDP broadcast (Winsock)
  Simulation.h/.cpp Motion model + emit cadence (background thread)
  main.cpp         Win32 dialog UI
  app.rc, resource.h  Dialog resource
test/
  verify.cpp       Encoder round-trip / checksum checks
  netcheck.cpp     End-to-end TCP streaming smoke test
```

## Tests

Both test programs build from a VS x64 developer command prompt:

```powershell
cl /EHsc /std:c++17 /I src test\verify.cpp   src\Ais.cpp src\Nmea.cpp
cl /EHsc /std:c++17 /I src test\netcheck.cpp src\Simulation.cpp src\Ais.cpp src\Nmea.cpp src\Network.cpp
```

`verify.exe` decodes the generated AIS payloads back to their fields (MMSI,
lat/lon, SOG, IMO, message type) and validates every checksum. `netcheck.exe`
starts the server + simulation, connects a TCP client, and confirms a live
stream of GGA/RMC/AIVDM sentences.
