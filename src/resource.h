// resource.h - Control identifiers for the main dialog.
#pragma once

#define IDD_MAIN            100
#define IDI_APP             101

// Networking
#define IDC_TCP_PORT        1001
#define IDC_UDP_PORT        1002
#define IDC_START           1003
#define IDC_STATUS          1004

// Ownship
#define IDC_OWN_LAT         1010
#define IDC_OWN_LON         1011
#define IDC_OWN_WIDTH       1012
#define IDC_OWN_HEIGHT      1013
#define IDC_OWN_SHAPE       1014
#define IDC_OWN_SPEED       1015

// AIS targets - each target i uses base 1100 + i*10
#define IDC_T_BASE          1100
#define IDC_T_ENABLE        0   // +offset within a target block
#define IDC_T_CLASS         1
#define IDC_T_SHAPE         2
#define IDC_T_OFFX          3
#define IDC_T_OFFY          4
#define IDC_T_SPEED         5

#define TARGET_ID(i, field) (IDC_T_BASE + (i) * 10 + (field))

// Log
#define IDC_LOG             1200
#define IDC_CLEAR_LOG       1201
