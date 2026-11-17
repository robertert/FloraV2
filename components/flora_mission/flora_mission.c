/**
 * @file flora_mission.c
 * @brief Maszyna stanow doniczki, opis w flora_mission.h.
 */
#include <string.h>

#include "flora_mission.h"

void flora_mission_init(flora_mission_t *m, const flora_mission_cfg_t *cfg,
                        const flora_seek_cfg_t *seek_cfg)
{
    memset(m, 0, sizeof *m);
    m->cfg = *cfg;
    flora_seek_init(&m->seek, seek_cfg);
    m->last.state = FLORA_MISSION_IDLE;
    m->last.water_ok = true;
    m->last.may_sleep = true;
}

void flora_mission_set_seek_cfg(flora_mission_t *m, const flora_seek_cfg_t *seek_cfg)
{
    flora_seek_set_cfg(&m->seek, seek_cfg);
}

void flora_mission_reset(flora_mission_t *m, uint32_t now_s)
{
    m->fault = FLORA_MISSION_FAULT_NONE;
    m->blocked_seeks = 0;
    flora_seek_restart(&m->seek, now_s);
}

void flora_mission_force_seek(flora_mission_t *m)
{
    flora_seek_force(&m->seek);
}

static bool seek_active(const flora_seek_t *s)
{
    return s->state != FLORA_SEEK_WATCH;
}

flora_mission_out_t flora_mission_update(flora_mission_t *m, const flora_mission_in_t *in)
{
    flora_mission_out_t o = { .cmd = FLORA_SEEK_CMD_NONE };
    const uint32_t now = in->seek.now_s;

    if (!in->enabled) {
        /* Wylaczenie w trakcie jazdy: zatrzymac to, co misja sama zlecila. */
        if (m->was_enabled && seek_active(&m->seek) &&
            in->seek.nav == FLORA_SEEK_NAV_BUSY) {
            o.cmd = FLORA_SEEK_CMD_STOP;
        }
        if (m->was_enabled) {
            flora_seek_restart(&m->seek, now);
        }
        m->was_enabled = false;
        o.state = FLORA_MISSION_IDLE;
        o.fault = m->fault;
        o.water_ok = true;
        o.may_sleep = true;
        o.seek = m->seek.last;
        m->last = o;
        return o;
    }
    if (!m->was_enabled) {
        /* Wlaczenie: czysta karta, postoj liczony od teraz. */
        m->was_enabled = true;
        flora_mission_reset(m, now);
    }

    if (in->stalled && m->fault == FLORA_MISSION_FAULT_NONE) {
        m->fault = FLORA_MISSION_FAULT_STALL;
    }

    const bool faulted = m->fault != FLORA_MISSION_FAULT_NONE;
    const bool held = !in->autonomy_ok || in->manual || !in->pose_ok;
    /* Dawka w toku wstrzymuje tylko START szukania - w trakcie jazdy
     * podlewanie i tak nie ma zgody, wiec nie moze trwac. */
    const bool hold = faulted || held || !in->drive_allowed ||
                      (in->water_busy && !seek_active(&m->seek));

    flora_seek_in_t si = in->seek;
    si.hold = hold;
    const flora_seek_out_t so = flora_seek_update(&m->seek, &si);

    o.cmd = so.cmd;
    o.gx = so.gx;
    o.gy = so.gy;

    if (so.result == FLORA_SEEK_RES_BLOCKED) {
        if (++m->blocked_seeks >= m->cfg.max_blocked_seeks &&
            m->fault == FLORA_MISSION_FAULT_NONE) {
            m->fault = FLORA_MISSION_FAULT_BLOCKED;
        }
    } else if (so.result == FLORA_SEEK_RES_IMPROVED || so.result == FLORA_SEEK_RES_NO_GAIN) {
        m->blocked_seeks = 0;
    }

    if (m->fault != FLORA_MISSION_FAULT_NONE) {
        o.state = FLORA_MISSION_FAULT;
    } else if (so.state == FLORA_SEEK_MOVING || so.state == FLORA_SEEK_RETURN) {
        o.state = FLORA_MISSION_SEEK_MOVE;
    } else if (so.state == FLORA_SEEK_MEASURE) {
        o.state = FLORA_MISSION_SEEK_MEASURE;
    } else if (in->water_busy) {
        o.state = FLORA_MISSION_WATER;
    } else if (!in->drive_allowed) {
        o.state = FLORA_MISSION_LOW_BATT;
    } else if (held) {
        o.state = FLORA_MISSION_HOLD;
    } else {
        o.state = FLORA_MISSION_DWELL;
    }

    const bool moving = o.state == FLORA_MISSION_SEEK_MOVE ||
                        o.state == FLORA_MISSION_SEEK_MEASURE;
    o.fault = m->fault;
    o.water_ok = !moving;
    /* Reczna jazda i podlewanie trzymaja robota na jawie. */
    o.may_sleep = !moving && !in->water_busy && !in->manual;
    o.seek = so;
    m->last = o;
    return o;
}
