#ifndef __SETTINGS_H
#define __SETTINGS_H

#include <stdint.h>

/* ---- Magic byte ---- */
#define SETTINGS_MAGIC 0xAD  /* magic byte to distinguish valid settings from erased flash */

/* ---- Preset slots ---- */
#define PRESET_SLOTS 5

typedef struct {
    uint32_t voltage_mv;
    uint32_t current_ma;
    char name[8];
} Preset_t;

/* ---- Settings struct (stored in flash with CRC) ---- */
typedef struct {
    uint8_t magic;               /* SETTINGS_MAGIC — validates flash contents */
    /* Mode */
    uint8_t remember_boot;       /* 0=NO default, 1=YES */
    uint8_t boot_selector;       /* 1=YES default, 0=NO */
    uint8_t serial_terminal;     /* 1=YES default, 0=NO */
    uint8_t splash_screen;       /* 1=YES default, 0=NO */
    /* Sound */
    uint8_t buzzer_enabled;      /* 1=YES default */
    uint8_t startup_beep;        /* 1=YES default */
    /* Display */
    uint8_t temp_fahrenheit;     /* 0=Celsius (default), 1=Fahrenheit */
    /* No padding needed: magic + 6 mode/sound bytes + 1 display byte = 8, aligned */
    /* Runtime (not in settings menu, but persisted) */
    uint32_t last_voltage_mv;
    uint32_t last_current_ma;
    uint8_t  last_used_pdo;      /* 1-based */
    uint8_t  preset_count;
    uint8_t  _pad2[2];
    /* Protection thresholds */
    uint16_t ocp_ma;             /* Over-current limit in mA, default 5500 */
    uint16_t ovp_mv;             /* Over-voltage limit in mV, default 55000 */
    uint16_t opp_100mw;          /* Over-power limit in units of 100mW, default 1400 (=140W) */
    uint8_t  ocp_retry;          /* OCP retry policy: 0=latch, 1=retry-once, 2=retry-3x */
    uint8_t  _pad3;              /* alignment padding */
    /* Timer auto-shutoff */
    uint16_t timer_seconds;      /* Auto-shutoff timer in seconds, 0=disabled */
    /* Ah/Wh limits */
    uint32_t ah_limit_mah;       /* Charge limit in mAh, 0=disabled */
    uint32_t wh_limit_mwh;       /* Energy limit in mWh, 0=disabled */
    uint16_t charge_complete_ma;   /* current threshold in mA, 0=disabled */
    uint16_t charge_complete_sec;  /* hold time in seconds before shutoff */
    /* Calibration offsets — signed, applied after raw ADC read */
    int16_t cal_v_offset_uv;     /* voltage offset in microvolts, default 0 */
    int16_t cal_i_offset_ua;     /* current offset in microamps, default 0  */
    /* Display — graph */
    uint8_t graph_window;        /* Graph time window: 0=5s, 1=10s (default), 2=20s */
    uint8_t _pad4[1];            /* alignment padding */
    /* Presets */
    Preset_t presets[PRESET_SLOTS];
} Settings_t;

/* ---- Menu item identifiers ---- */
enum {
    /* Mode group (0-99) */
    MI_REMEMBER_BOOT    = 0,
    MI_BOOT_SELECTOR    = 1,
    MI_SERIAL_TERMINAL  = 2,
    MI_SPLASH_SCREEN    = 3,
    /* Display group (50-99) */
    MI_TEMP_UNIT        = 50,
    MI_GRAPH_WINDOW     = 51,
    /* Sound group (100-199) */
    MI_BUZZER_ENABLED   = 100,
    MI_STARTUP_BEEP     = 101,
    /* Protection group (200-299) — numeric settings */
    MI_OCP_LIMIT        = 200,
    MI_OVP_LIMIT        = 201,
    MI_OPP_LIMIT        = 202,
    MI_TIMER            = 203,
    MI_AH_LIMIT         = 204,
    MI_WH_LIMIT         = 205,
    MI_OCP_RETRY        = 206,
    MI_CHARGE_COMPLETE_MA  = 208,
    MI_CHARGE_COMPLETE_SEC = 209,
    /* Tools group (800-899) — diagnostic tools, no flash field */
    MI_TOOL_CHARGER_INFO  = 800,
    MI_TOOL_VOLTAGE_SWEEP = 801,
    MI_TOOL_CABLE_INFO    = 802,
    MI_TOOL_SELFTEST      = 803,
    /* Calibration group (900-999) — signed offset adjustments */
    MI_CAL_V              = 900,
    MI_CAL_I              = 901,
    /* System group (700-799) — actions, no flash field */
    MI_LOAD_DEFAULT     = 700,
    MI_SAVE_REBOOT      = 701,
    MI_EXIT_NO_SAVE     = 702,
};

#define MI_NO_FLASH 0xFF  /* system actions have no flash field */

/* ---- Menu structures ---- */
typedef struct {
    uint16_t mi;          /* MI_ identifier */
    uint8_t  fi;          /* field index into Settings_t cast as uint8_t*, or MI_NO_FLASH */
    const char *name;     /* display string */
} MI_Entry;

typedef struct {
    const char *title;
    const uint16_t *items;  /* array of MI_ identifiers */
    uint8_t count;          /* number of items (excluding implicit "Back") */
} MenuGroup;

/* ---- Public API ---- */
void     Settings_Init(void);
void     Settings_Save(void);

/* Deferred save — flash erase stalls the CPU for ~50-100ms, which kills
 * EPR KeepAlive (500ms deadline).  Call Settings_ProcessDeferred() from
 * the main loop; it only writes when the PD stack is idle. */
void     Settings_ProcessDeferred(void);
uint8_t  Settings_IsSavePending(void);
void     Settings_LoadDefaults(void);

/* Bool settings accessors */
uint8_t  Settings_GetRememberBoot(void);
uint8_t  Settings_GetBootSelector(void);
uint8_t  Settings_GetSerialTerminal(void);
uint8_t  Settings_GetSplashScreen(void);
uint8_t  Settings_GetBuzzerEnabled(void);
uint8_t  Settings_GetStartupBeep(void);
uint8_t  Settings_GetTempFahrenheit(void);
void     Settings_SetTempFahrenheit(uint8_t val);
uint8_t  Settings_GetGraphWindow(void);
void     Settings_SetGraphWindow(uint8_t val);

void     Settings_SetBool(uint8_t fi, uint8_t val);
uint8_t  Settings_GetBool(uint8_t fi);

/* Runtime state */
uint32_t Settings_GetLastVoltage(void);
uint32_t Settings_GetLastCurrent(void);
void     Settings_SaveLastSettings(uint32_t mv, uint32_t ma);
uint8_t  Settings_GetLastUsedPdo(void);
void     Settings_SetLastUsedPdo(uint8_t pdo);

/* Preset accessors */
Preset_t* Settings_GetPreset(uint8_t index);
uint8_t  Settings_PresetCount(void);
uint8_t  Settings_PresetIsEmpty(uint8_t index);
void     Settings_PresetSet(uint8_t index, uint32_t mv, uint32_t ma, const char *name);
void     Settings_PresetDelete(uint8_t index);

/* Numeric settings accessors */
uint16_t Settings_GetOcpMa(void);
uint16_t Settings_GetOvpMv(void);
uint16_t Settings_GetOpp100mw(void);
uint16_t Settings_GetTimerSeconds(void);
uint32_t Settings_GetAhLimitMah(void);
uint32_t Settings_GetWhLimitMwh(void);
uint8_t  Settings_GetOcpRetry(void);
void     Settings_SetOcpRetry(uint8_t val);
uint16_t Settings_GetChargeCompleteMa(void);
uint16_t Settings_GetChargeCompleteSec(void);
int16_t  Settings_GetCalVOffsetUv(void);
int16_t  Settings_GetCalIOffsetUa(void);
void     Settings_SetNumeric(uint16_t mi, int32_t delta);
uint32_t Settings_GetNumericValue(uint16_t mi);

/* Menu data (defined in menu_settings.c) */
extern const MI_Entry   g_mi_table[];
extern const uint8_t    g_mi_table_size;
extern const MenuGroup  g_menu_groups[];
extern const uint8_t    g_menu_group_count;

/* Menu helpers */
const MI_Entry* Menu_FindMI(uint16_t mi);
const char*     Menu_GetValueStr(uint16_t mi);
void            Menu_ToggleBool(uint16_t mi);
void            Menu_AdjustNumeric(uint16_t mi, int8_t direction); /* +1 or -1 */
uint8_t         Menu_IsNumeric(uint16_t mi);  /* returns 1 for numeric items */

#endif
