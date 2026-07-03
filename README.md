# NMEA Generator

A Windows desktop application that generates **NMEA 0183** or **NMEA 2000** test
data and serves it simultaneously over a **TCP server** and **UDP broadcast**.
The protocol is selected with a UI toggle; only one mode is active at a time.

## Features

### Protocol Modes

The **Protocol** selector in the Networking group chooses the active output and
input format.

| Mode | Output |
|------|--------|
| `NMEA 0183` | Standard text sentences (`GGA`, `RMC`, `AIVDM`, etc.) |
| `NMEA 2000` | Binary PGN payloads transported as Actisense N2K ASCII lines |

NMEA 2000 mode emulates an Actisense device using the documented **N2K ASCII**
format:

```text
Ahhmmss.ddd <SS><DD><P> <PPPPP> <payload-hex><CR><LF>
```

Fields are:

| Field | Meaning |
|-------|---------|
| `A` | Actisense N2K ASCII received-message identifier |
| `hhmmss.ddd` | transmission/reception time; milliseconds are included |
| `SS` | source address, hex |
| `DD` | destination address, hex; `FF` is global |
| `P` | priority, hex `0..7` |
| `PPPPP` | PGN, hex |
| `payload-hex` | reassembled PGN payload bytes, two hex digits per byte |

There is no NMEA `$...*hh` checksum in this format. The payload is the
reassembled NMEA 2000 PGN data, so consumers do not need to perform Fast-Packet
or ISO Transport Protocol reassembly.

### Ownship

Position is driven around a configurable path (circle / square / figure-eight)
centred on a latitude/longitude you choose, within a simulation box sized in
nautical miles. COG/SOG are derived from motion; apparent wind is computed from
a synthesised true wind. Ownship heading is rate-limited to **6 deg/s**, so turns
sweep round over time rather than snapping instantly.

In NMEA 0183 mode, ownship emits these sentences at ~1 Hz:

| Sentence | Content |
|----------|---------|
| `GGA` | GPS fix |
| `RMC` | Position, COG, SOG, date |
| `VTG` | Course and speed over ground |
| `GLL` | Geographic position |
| `VHW` | Heading and speed through water |
| `MWV` (R) | Apparent wind |
| `MWV` (T) | True wind |
| `MWD` | True wind direction and speed |

In NMEA 2000 mode, ownship emits equivalent PGNs:

| PGN | Content |
|-----|---------|
| `126992` | System time |
| `127250` | Vessel heading |
| `128259` | Speed, water-referenced |
| `129025` | Position rapid update |
| `129026` | COG/SOG rapid update |
| `129029` | GNSS position data |
| `130306` | Wind data (apparent and true) |

### Autopilot Input

The generator listens on the same TCP connections and UDP port for inbound
autopilot data. In NMEA 0183 mode, it accepts **APB**, **RMB**, and **XTE**. In
NMEA 2000 mode, it accepts equivalent Actisense N2K ASCII PGNs:

| PGN | Autopilot content |
|-----|-------------------|
| `129283` | Cross Track Error |
| `127237` | Heading/Track Control |
| `129284` | Navigation Data (destination, bearing, range) |

Valid input switches the ownship out of its predefined pattern and emulates an
autopilot, steering toward the destination waypoint or commanded heading. If no
autopilot data arrives for 15 seconds, the ownship resumes the predefined path.

Every inbound APB/RMB/XTE sentence and supported NMEA 2000 autopilot PGN is
checked before it is acted on. Errors are reported in the log and malformed data
is ignored.

Validation includes:

| Format | Checks |
|--------|--------|
| NMEA 0183 | leading `$`, checksum, field count, status flags, numeric fields, units/reference fields, steer direction, destination lat/lon range |
| NMEA 2000 Actisense ASCII | message type `A`, timestamp, `<SS><DD><P>` header, hex PGN, hex payload, complete payload length, decoded range checks |

Decoded values shown in the log include Cross Track Error, Heading to Steer,
Destination latitude/longitude, Bearing to destination, and Range to destination
when the input carries them.

Example decoded lines:

```text
RMB OK   Cross-track 0.10 NM (steer L) | Dest 50deg30.00'N 000deg30.00'W | Bearing-to-dest 045.0degT | Range-to-dest 12.50 NM
N2K 129284 OK   Dest 50deg30.00'N 000deg30.00'W | Bearing-to-dest 045.0degT | Range-to-dest 12.50 NM
```

### Log Colours

| Colour | Meaning |
|--------|---------|
| black | generated outbound data |
| blue | raw incoming data |
| green | valid decoded autopilot data |
| red | malformed autopilot data and the reason |
| magenta | autopilot engaged/lost status notices |

Echoes of the generator's own output received back via UDP broadcast are ignored.

### AIS Targets

Each of the four targets travels its own path with an optional X/Y offset from
the simulation centre. Targets can be Class A, Class B, SAR fixed-wing aircraft,
or SAR helicopter and toggled on/off individually.

Dynamic data is emitted every 6 seconds:

| Mode | Class | Output |
|------|-------|--------|
| NMEA 0183 | A | AIS type 1 |
| NMEA 0183 | B | AIS type 18 |
| NMEA 0183 | SAR fixed-wing / helicopter | AIS type 9 |
| NMEA 2000 | A | PGN 129038 |
| NMEA 2000 | B | PGN 129039 |
| NMEA 2000 | SAR fixed-wing / helicopter | PGN 129798 |

Static data is emitted every 60 seconds:

| Mode | Class | Output |
|------|-------|--------|
| NMEA 0183 | A | AIS type 5 |
| NMEA 0183 | B | AIS type 24 |
| NMEA 2000 | A | PGN 129794 |
| NMEA 2000 | B | PGNs 129809 and 129810 |

MMSI, IMO, name, call sign, ship type, and dimensions are pseudo-randomly but
deterministically generated per target.

SAR aircraft MMSIs use MID `366` by default. Fixed-wing targets use
`111366100..111366499`; helicopters use `111366500..111366999`.

### MOB Broadcast

The **Send MOB** button starts a 2-minute AIS MOB burst at the current ownship
position. Every 6 seconds the app sends:

| Mode | Output |
|------|--------|
| NMEA 0183 | AIS message 1 position report and AIS message 14 safety broadcast (`MOB ACTIVE`) |
| NMEA 2000 | PGN 129038 position report and PGN 129802 safety broadcast |

The MOB source MMSI is `972366001`.

## Building

Requires Windows 10/11 and Visual Studio with the C++ desktop workload.

Open `NMEA-Generator.sln` and build x64 Debug or Release.

The platform toolset is selected automatically:

| Visual Studio | Toolset |
|---------------|---------|
| 2026 | `v145` |
| 2022 | `v143` |

Command line:

```powershell
& "<VS>\MSBuild\Current\Bin\MSBuild.exe" NMEA-Generator.sln /p:Configuration=Release /p:Platform=x64
```

The executable is written to `build\x64\Release\NMEA-Generator.exe`.

There are no external dependencies. The app uses only the Win32 API and Winsock
(`Ws2_32.lib`). Qt is not required.

## Using It

1. Set the TCP port to listen on and the UDP port to broadcast on.
2. Choose **NMEA 0183** or **NMEA 2000** in the Protocol selector.
3. Configure the ownship centre, area size, path shape, and speed.
4. Enable/configure AIS targets.
5. Click **Start Simulation**.
6. Use **Send MOB** while running to start a 2-minute MOB broadcast.

Generated lines appear in the log, stream to every connected TCP client, and are
broadcast over UDP.

Settings are saved automatically to `%APPDATA%\NMEA-Generator\settings.ini` when
the simulation starts/stops or the app closes, and are restored on next launch.

Quick local TCP check:

```powershell
$c = New-Object System.Net.Sockets.TcpClient('127.0.0.1', 10110)
$r = New-Object System.IO.StreamReader($c.GetStream())
while ($true) { $r.ReadLine() }
```

## Project Layout

```text
NMEA-Generator.sln
NMEA-Generator.vcxproj
src/
  Geo.h              Path geometry and lat/lon conversion
  Nmea.h/.cpp        NMEA 0183 sentence building, checksum, parse helpers
  N2k.h/.cpp         NMEA 2000 PGN payloads via Actisense N2K ASCII
  Ais.h/.cpp         AIS 6-bit payload encoding (types 1/5/18/24)
  ApInput.h/.cpp     APB/RMB/XTE validation and decoding
  Network.h/.cpp     TCP server, UDP broadcast, inbound receive
  Simulation.h/.cpp  Motion model, turn-rate limit, autopilot, emit cadence
  main.cpp           Win32 dialog UI with colour-coded RichEdit log
  app.rc             Dialog resource
test/
  verify.cpp         Encoder/checksum/parser checks
  netcheck.cpp       NMEA 0183 TCP streaming smoke test
  n2kcheck.cpp       NMEA 2000 generation, validation, autopilot test
  apcheck.cpp        NMEA 0183 autopilot steering test
  turncheck.cpp      6 deg/s turn-rate test
```

## Tests

From a VS x64 developer command prompt:

```powershell
cl /utf-8 /EHsc /std:c++17 /I src test\verify.cpp    src\Ais.cpp src\Nmea.cpp src\ApInput.cpp src\N2k.cpp
cl /utf-8 /EHsc /std:c++17 /I src test\netcheck.cpp  src\Simulation.cpp src\Ais.cpp src\Nmea.cpp src\Network.cpp src\ApInput.cpp src\N2k.cpp
cl /utf-8 /EHsc /std:c++17 /I src test\n2kcheck.cpp  src\N2k.cpp src\Simulation.cpp src\Ais.cpp src\Nmea.cpp src\Network.cpp src\ApInput.cpp
cl /utf-8 /EHsc /std:c++17 /I src test\apcheck.cpp   src\Simulation.cpp src\Ais.cpp src\Nmea.cpp src\Network.cpp src\ApInput.cpp src\N2k.cpp
cl /utf-8 /EHsc /std:c++17 /I src test\turncheck.cpp src\Simulation.cpp src\Ais.cpp src\Nmea.cpp src\Network.cpp src\ApInput.cpp src\N2k.cpp
```

`n2kcheck.exe` verifies Actisense N2K ASCII parsing, ownship and AIS PGN generation,
autopilot PGN validation/decoding, and live steering from PGN 129284.
