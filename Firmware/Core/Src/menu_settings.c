/**
 * @file    menu_settings.c
 * @brief   Menu item table and group definitions for AxxPD settings UI
 */

#include "settings.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/*  MI table — maps each MI_ identifier to its field offset and name  */
/* ------------------------------------------------------------------ */
const MI_Entry g_mi_table[] = {
    /* Mode group */
    { MI_REMEMBER_BOOT,   (uint8_t)offsetof(Settings_t, remember_boot),   "Restore last V/I" },
    { MI_POWER_ON_BOOT,   (uint8_t)offsetof(Settings_t, power_on_boot),   "Power on boot"   },
    { MI_START_LOCKED,    (uint8_t)offsetof(Settings_t, start_locked),    "Start locked"    },
    { MI_BOOT_SELECTOR,   (uint8_t)offsetof(Settings_t, boot_selector),   "Boot PDO select"  },
    { MI_SERIAL_TERMINAL, (uint8_t)offsetof(Settings_t, serial_terminal), "Serial terminal"  },
    { MI_SPLASH_SCREEN,   (uint8_t)offsetof(Settings_t, splash_screen),   "Splash screen"    },
    /* Display group */
    { MI_TEMP_UNIT,       (uint8_t)offsetof(Settings_t, temp_fahrenheit), "Temp unit"        },
    { MI_GRAPH_WINDOW,    MI_NO_FLASH,                                    "Graph window"     },
    /* Sound group */
    { MI_BUZZER_ENABLED,  (uint8_t)offsetof(Settings_t, buzzer_enabled),  "Buzzer"           },
    { MI_STARTUP_BEEP,    (uint8_t)offsetof(Settings_t, startup_beep),    "Startup beep"     },
    /* Protection group — numeric items; fi=MI_NO_FLASH (value via Settings_GetNumericValue) */
    { MI_OCP_LIMIT,       MI_NO_FLASH,                                    "OCP limit"        },
    { MI_OVP_LIMIT,       MI_NO_FLASH,                                    "OVP limit"        },
    { MI_OPP_LIMIT,       MI_NO_FLASH,                                    "OPP limit"        },
    { MI_TIMER,           MI_NO_FLASH,                                    "Timer shutoff"    },
    { MI_AH_LIMIT,        MI_NO_FLASH,                                    "Ah limit"         },
    { MI_WH_LIMIT,        MI_NO_FLASH,                                    "Wh limit"         },
    { MI_OCP_RETRY,       MI_NO_FLASH,                                    "OCP retry"        },
    { MI_CHARGE_COMPLETE_MA,  MI_NO_FLASH,                               "CC Thresh"        },
    { MI_CHARGE_COMPLETE_SEC, MI_NO_FLASH,                               "CC Hold"          },
    /* Tools group (diagnostic — no flash field) */
    { MI_TOOL_CHARGER_INFO,  MI_NO_FLASH,                                 "Charger info"     },
    { MI_TOOL_CABLE_INFO,    MI_NO_FLASH,                                 "Cable info"       },
    { MI_TOOL_SELFTEST,      MI_NO_FLASH,                                 "Self-test"        },
    /* Calibration group — signed numeric offsets in mV and mA */
    { MI_CAL_V,              MI_NO_FLASH,                                 "V offset"         },
    { MI_CAL_I,              MI_NO_FLASH,                                 "I offset"         },
    /* System group (actions — no flash field) */
    { MI_LOAD_DEFAULT,    MI_NO_FLASH,                                    "Load defaults"    },
    { MI_SAVE_REBOOT,     MI_NO_FLASH,                                    "Save & Reboot"    },
    { MI_EXIT_NO_SAVE,    MI_NO_FLASH,                                    "Exit"             },
};

const uint8_t g_mi_table_size = sizeof(g_mi_table) / sizeof(g_mi_table[0]);

/* ------------------------------------------------------------------ */
/*  Menu groups                                                        */
/* ------------------------------------------------------------------ */
static const uint16_t mode_items[] = {
    MI_REMEMBER_BOOT, MI_POWER_ON_BOOT, MI_START_LOCKED, MI_BOOT_SELECTOR, MI_SERIAL_TERMINAL, MI_SPLASH_SCREEN
};

static const uint16_t display_items[] = {
    MI_TEMP_UNIT, MI_GRAPH_WINDOW
};

static const uint16_t sound_items[] = {
    MI_BUZZER_ENABLED, MI_STARTUP_BEEP
};

static const uint16_t protection_items[] = {
    MI_OCP_LIMIT, MI_OVP_LIMIT, MI_OPP_LIMIT, MI_TIMER, MI_AH_LIMIT, MI_WH_LIMIT,
    MI_OCP_RETRY, MI_CHARGE_COMPLETE_MA, MI_CHARGE_COMPLETE_SEC
};

static const uint16_t tools_items[] = {
    MI_TOOL_CHARGER_INFO, MI_TOOL_CABLE_INFO, MI_TOOL_SELFTEST
};

static const uint16_t calibration_items[] = {
    MI_CAL_V, MI_CAL_I
};

static const uint16_t system_items[] = {
    MI_LOAD_DEFAULT, MI_SAVE_REBOOT, MI_EXIT_NO_SAVE
};

const MenuGroup g_menu_groups[] = {
    { "Mode",       mode_items,       sizeof(mode_items)       / sizeof(mode_items[0])       },
    { "Display",    display_items,    sizeof(display_items)    / sizeof(display_items[0])    },
    { "Sound",      sound_items,      sizeof(sound_items)      / sizeof(sound_items[0])      },
    { "Protection", protection_items, sizeof(protection_items) / sizeof(protection_items[0]) },
    { "Tools",      tools_items,      sizeof(tools_items)      / sizeof(tools_items[0])      },
    { "Calibration", calibration_items, sizeof(calibration_items) / sizeof(calibration_items[0]) },
    { "System",     system_items,     sizeof(system_items)     / sizeof(system_items[0])     },
};

const uint8_t g_menu_group_count = sizeof(g_menu_groups) / sizeof(g_menu_groups[0]);

/* ------------------------------------------------------------------ */
/*  Menu helpers                                                       */
/* ------------------------------------------------------------------ */
const MI_Entry* Menu_FindMI(uint16_t mi)
{
    for (uint8_t i = 0; i < g_mi_table_size; i++) {
        if (g_mi_table[i].mi == mi) {
            return &g_mi_table[i];
        }
    }
    return NULL;
}

/** Returns 1 if the menu item is a numeric (adjustable) setting. */
uint8_t Menu_IsNumeric(uint16_t mi)
{
    return (mi == MI_OCP_LIMIT || mi == MI_OVP_LIMIT || mi == MI_OPP_LIMIT ||
            mi == MI_TIMER     || mi == MI_AH_LIMIT  || mi == MI_WH_LIMIT  ||
            mi == MI_OCP_RETRY || mi == MI_CAL_V     || mi == MI_CAL_I     ||
            mi == MI_CHARGE_COMPLETE_MA || mi == MI_CHARGE_COMPLETE_SEC ||
            mi == MI_GRAPH_WINDOW);
}

const char* Menu_GetValueStr(uint16_t mi)
{
    /* Numeric items — format with units into a static buffer */
    if (Menu_IsNumeric(mi)) {
        static char nbuf[20];
        switch (mi) {
            case MI_OCP_LIMIT: {
                uint32_t ma = Settings_GetOcpMa();
                snprintf(nbuf, sizeof(nbuf), "%lu.%luA", ma / 1000UL, (ma % 1000UL) / 100UL);
                return nbuf;
            }
            case MI_OVP_LIMIT: {
                uint32_t mv = Settings_GetOvpMv();
                snprintf(nbuf, sizeof(nbuf), "%luV", mv / 1000UL);
                return nbuf;
            }
            case MI_OPP_LIMIT: {
                uint32_t w = (uint32_t)Settings_GetOpp100mw() / 10UL;
                snprintf(nbuf, sizeof(nbuf), "%luW", w);
                return nbuf;
            }
            case MI_TIMER: {
                uint32_t s = Settings_GetTimerSeconds();
                if (s == 0) return "OFF";
                snprintf(nbuf, sizeof(nbuf), "%lus", s);
                return nbuf;
            }
            case MI_AH_LIMIT: {
                uint32_t mah = Settings_GetAhLimitMah();
                if (mah == 0) return "OFF";
                snprintf(nbuf, sizeof(nbuf), "%lu.%luAh", mah / 1000UL, (mah % 1000UL) / 100UL);
                return nbuf;
            }
            case MI_WH_LIMIT: {
                uint32_t mwh = Settings_GetWhLimitMwh();
                if (mwh == 0) return "OFF";
                snprintf(nbuf, sizeof(nbuf), "%luWh", mwh / 1000UL);
                return nbuf;
            }
            case MI_OCP_RETRY: {
                uint8_t r = Settings_GetOcpRetry();
                if (r == 0) return "Latch";
                if (r == 1) return "1 retry";
                return "3 retries";
            }
            case MI_GRAPH_WINDOW: {
                switch (Settings_GetGraphWindow()) {
                    case 0:  return "5s";
                    case 2:  return "30s";
                    case 3:  return "60s";
                    default: return "10s";
                }
            }
            case MI_CHARGE_COMPLETE_MA: {
                uint32_t ma = Settings_GetChargeCompleteMa();
                if (ma == 0) return "OFF";
                snprintf(nbuf, sizeof(nbuf), "%lumA", ma);
                return nbuf;
            }
            case MI_CHARGE_COMPLETE_SEC: {
                uint32_t s = Settings_GetChargeCompleteSec();
                snprintf(nbuf, sizeof(nbuf), "%lus", s);
                return nbuf;
            }
            case MI_CAL_V: {
                float mv = (float)Settings_GetCalVOffsetUv() / 1000.0f;
                snprintf(nbuf, sizeof(nbuf), "%+.1fmV", (double)mv);
                return nbuf;
            }
            case MI_CAL_I: {
                float ma = (float)Settings_GetCalIOffsetUa() / 1000.0f;
                snprintf(nbuf, sizeof(nbuf), "%+.1fmA", (double)ma);
                return nbuf;
            }
            default: return "--";
        }
    }

    /* Temperature unit — display "C" or "F" instead of YES/NO */
    if (mi == MI_TEMP_UNIT) {
        return Settings_GetTempFahrenheit() ? "F" : "C";
    }

    const MI_Entry *entry = Menu_FindMI(mi);
    if (!entry) return "--";

    /* System actions have no value to display */
    if (entry->fi == MI_NO_FLASH) return "--";

    /* Bool field: read from RAM mirror via field offset */
    uint8_t val = Settings_GetBool(entry->fi);
    return val ? "YES" : "NO";
}

void Menu_ToggleBool(uint16_t mi)
{
    const MI_Entry *entry = Menu_FindMI(mi);
    if (!entry) return;
    if (entry->fi == MI_NO_FLASH) return;
    /* Numeric items are not toggled this way */
    if (Menu_IsNumeric(mi)) return;

    uint8_t val = Settings_GetBool(entry->fi);
    Settings_SetBool(entry->fi, val ? 0U : 1U);
    Settings_SaveDeferred();  /* coalesce rapid toggles into one flash write */
}

/** Increment (+1) or decrement (-1) a numeric setting by one step. */
void Menu_AdjustNumeric(uint16_t mi, int8_t direction)
{
    if (!Menu_IsNumeric(mi)) return;
    Settings_SetNumeric(mi, (int32_t)direction);
}
