// Inspired by https://blog.yavilevich.com/2020/09/convention-for-compile-time-configuration-of-platformio-projects/?utm_source=pocket_mylist
#pragma once

#ifndef THERMOSTAT_WIFI_SSID
#error THERMOSTAT_WIFI_SSID env variable is not set
#endif

#ifndef THERMOSTAT_WIFI_PASSWORD
#error THERMOSTAT_WIFI_PASSWORD env variable is not set
#endif

#ifndef THERMOSTAT_RELAY_PIN
#error THERMOSTAT_RELAY_PIN env variable is not set
#endif

#ifndef THERMOSTAT_SENSOR_PIN
#error THERMOSTAT_SENSOR_PIN env variable is not set
#endif

#ifndef THERMOSTAT_SIMULATING
#define THERMOSTAT_SIMULATING true
#endif

#ifndef THERMOSTAT_SERVER_PORT
#define THERMOSTAT_SERVER_PORT 80
#endif

#ifndef THERMOSTAT_TIMEZONE
// Most of Europe           "MET-1METDST,M3.5.0/01,M10.5.0/02";
// Central Europe           "CET-1CEST,M3.5.0,M10.5.0/3";
// Most of Europe           "EST-2METDST,M3.5.0/01,M10.5.0/02";
// EST USA                  "EST5EDT,M3.2.0,M11.1.0";
// CST USA                  "CST6CDT,M3.2.0,M11.1.0";
// MST USA                  "MST7MDT,M4.1.0,M10.5.0";
// Auckland                 "NZST-12NZDT,M9.5.0,M4.1.0/3";
// Asia                     "EET-2EEST,M3.5.5/0,M10.5.5/0";
// Australia                "ACST-9:30ACDT,M10.1.0,M4.1.0/3":
#define THERMOSTAT_TIMEZONE "MET-1METDST,M3.5.0/01,M10.5.0/02"
#endif
