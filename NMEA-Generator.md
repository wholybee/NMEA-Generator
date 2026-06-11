# NMEA Generator

## Goal

To build an application that generates NMEA 0183 test data with AIS information.

## Requirements

Generate NMEA 0183 Test data.

Act as both a TCP IP server and also broadcast via UDP.

### Generate Sentences for Ownship

Generate simulated GPS latitude and longitude for a vessel (Ownship)

Generate simulated COG, COG for Ownship

Generate simulated Speed through water for Ownship

Generate simulated Apparent Wind Speed and Direction for Ownship

Generate simulated True Wind Speed and Direction for Ownship

### Generate Sentences for 4 AIS targets

##### Generate AIS static data for each target, repeated every 60 seconds

MMSI Number

IMO Number

Vessel Name and Call Sign

Ship Type

Vessel Dimensions

AIS Class A/B

##### Generate AIS dynamic data for each target, repeated every 6 seconds

Position

SOG

COG

Heading

ROT

UTC

## User Interface

#### There will be a single dialog which will contain:

##### Networking

TCP port to listen on

UDP port to broadcast on

Turn simulation on/off

##### Ownship

Latitude and longitude of center of simulation

Size of simulation area in nautical miles, width (E-W) and height (N-S)

Shape of Ownship path, centered on center of simulation (circle, square, or figure-eight)

Speed of Ownship

##### AIS Targets

Each Target can be turned on/off individually

User can set Class A/B of each target

Each can have a different path (circle, square, figure eight)

Path can have x,y offset from center of simulation

Path MAY exceed the bounds of the simulation if x,y offset is applied

User can set speed of each target

COG, Heading, ROT are calculated

MMSI Number, IMO Number, Vessel Name and Call Sign, Ship Type, Vessel Dimensions are pseudo randomly generated.

## Target Build Environment

Application targets Windows 10/11

Application will be a Visual Studio Project

Visual Studio Version is 2026, with a build configuration for vs2022

QT6 is available if you want to use it (I don't care if you do or don't)





