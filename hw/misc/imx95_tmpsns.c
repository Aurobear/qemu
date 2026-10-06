/*
 * i.MX95 TMPSNS polling/conversion model for local firmware experiments.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Register contract: NXP MIMX95_TMPSNS.h and fsl_tmpsns.c. Temperature is an
 * explicit simulation input (millidegrees C), not a host temperature reading
 * or a thermal-physics model. Comparator filters, trip IRQs, calibration and
 * power-domain coupling are not implemented; do not use for safety validation.
 */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "migration/vmstate.h"

#define TYPE_IMX95_TMPSNS "imx95.tmpsns"
OBJECT_DECLARE_SIMPLE_TYPE(IMX95Tmpsns, IMX95_TMPSNS)

#define CTRL0       0x000
#define STAT0       0x010
#define DATA0       0x020
#define CTRL1       0x200
#define STAT1       0x210
#define DATA1       0x220
#define PERIOD      0x270
#define REF_DIV     0x280
#define ENABLE      (1u << 31)
#define START       (1u << 30)
#define STOP        (1u << 29)
#define READY       (1u << 16)
#define IDLE        (1u << 31)
#define REG_COUNT   (0x300 / 16)

struct IMX95Tmpsns {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    QEMUTimer timer;
    uint32_t regs[REG_COUNT];
    int64_t temperature;
    bool running;
    bool boot_enabled;
};

/* Functional timing: 24 MHz source, programmed divider and periodic count.
 * One-shot/continuous use the documented conversion times at a 4 MHz clock.
 * Analogue settling, filters and clock-tree rate changes are not simulated.
 */
static int64_t sample_delay(IMX95Tmpsns *s)
{
    static const int64_t conversion_ns[] = { 593250, 1105250, 2129250, 4177250 };
    uint32_t ctrl = s->regs[CTRL1 / 16];
    unsigned div = ((s->regs[REF_DIV / 16] >> 16) & 0xff) + 1;
    int64_t delay = conversion_ns[(ctrl >> 18) & 3] * div / 6;

    if (((ctrl >> 24) & 3) == 2) {
        int64_t period = (s->regs[PERIOD / 16] & 0xffffff) *
                         (int64_t)div * 1000000000 / 24000000;
        delay = MAX(delay, period);
    }
    return MAX(delay, 1);
}

static void sample(void *opaque)
{
    IMX95Tmpsns *s = opaque;
    uint32_t ctrl = s->regs[CTRL1 / 16];
    uint16_t raw;

    if (!s->running || !(ctrl & ENABLE)) {
        return;
    }
    raw = (uint16_t)(int16_t)(s->temperature * 64 / 1000);
    s->regs[DATA0 / 16] = raw;
    s->regs[DATA1 / 16] = raw;
    s->regs[STAT0 / 16] |= READY;
    s->regs[STAT1 / 16] |= READY;
    if (((ctrl >> 24) & 3) == 0) {
        s->running = false;
        s->regs[STAT1 / 16] |= IDLE;
    } else {
        timer_mod(&s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + sample_delay(s));
    }
}

static bool implemented(hwaddr reg)
{
    switch (reg) {
    case CTRL0: case STAT0: case DATA0: case 0x30: case 0x40:
    case CTRL1: case STAT1: case DATA1: case 0x250: case PERIOD:
    case REF_DIV: case 0x2b0: case 0x2c0: case 0x2e0: case 0x2f0:
        return true;
    default:
        return false;
    }
}

static uint64_t tmpsns_read(void *opaque, hwaddr offset, unsigned size)
{
    IMX95Tmpsns *s = opaque;
    hwaddr reg = offset & ~0xf;

    return implemented(reg) ? s->regs[reg / 16] : 0;
}

static void tmpsns_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    IMX95Tmpsns *s = opaque;
    hwaddr reg = offset & ~0xf;
    uint32_t *p;

    if (!implemented(reg) || reg == DATA0 || reg == DATA1 || reg == 0x2c0) {
        return;
    }
    p = &s->regs[reg / 16];
    /* Status flags are hardware-set and software-cleared via CLR aliases. */
    if (reg == STAT0 || reg == STAT1) {
        if ((offset & 0xf) == 8) {
            *p &= ~(value & READY);
        }
        return;
    }
    switch (offset & 0xf) {
    case 0: *p = value; break;
    case 4: *p |= value; break;
    case 8: *p &= ~value; break;
    case 12: *p ^= value; break;
    }
    if (reg == REF_DIV && !(*p & (1u << 31))) {
        s->running = false;
        timer_del(&s->timer);
        s->regs[STAT1 / 16] |= IDLE;
    }
    if (reg == CTRL0) {
        /* Filter reset commands self-clear; comparator IRQs are unsupported. */
        *p &= ~(7u << 20);
    }
    if (reg == CTRL1) {
        bool start = *p & START;
        bool stop = *p & STOP;
        *p &= ~(START | STOP | (3u << 20));
        if (!(*p & ENABLE) || stop || ((*p >> 24) & 3) == 3) {
            s->running = false;
            timer_del(&s->timer);
            s->regs[STAT1 / 16] |= IDLE;
        } else if (start && (s->regs[REF_DIV / 16] & (1u << 31))) {
            s->running = true;
            s->regs[STAT0 / 16] &= ~READY;
            s->regs[STAT1 / 16] &= ~(READY | IDLE);
            timer_mod(&s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + sample_delay(s));
        }
    }
}

static const MemoryRegionOps tmpsns_ops = {
    .read = tmpsns_read,
    .write = tmpsns_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void temperature_get(Object *obj, Visitor *v, const char *name,
                            void *opaque, Error **errp)
{
    int64_t value = IMX95_TMPSNS(obj)->temperature;
    visit_type_int64(v, name, &value, errp);
}

static void temperature_set(Object *obj, Visitor *v, const char *name,
                            void *opaque, Error **errp)
{
    int64_t value;
    if (!visit_type_int64(v, name, &value, errp)) {
        return;
    }
    if (value < -40000 || value > 125000) {
        error_setg(errp, "temperature must be in [-40000, 125000] millidegrees C");
        return;
    }
    IMX95_TMPSNS(obj)->temperature = value;
}

static void tmpsns_reset(DeviceState *dev)
{
    IMX95Tmpsns *s = IMX95_TMPSNS(dev);
    timer_del(&s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[STAT1 / 16] = IDLE;
    s->running = false;
    if (s->boot_enabled) {
        /* ANA is configured before SM by the skipped ROM/ELE boot stage.
         * SM calls SensorConfigStart(ANA, false), so it cannot program CTRL1.
         * Supply that documented boot prerequisite, not a forged ready flag.
         */
        s->regs[REF_DIV / 16] = 0x80050000;
        s->regs[PERIOD / 16] = 100000;
        s->regs[CTRL1 / 16] = ENABLE | (2u << 24) | (1u << 18);
        tmpsns_write(s, CTRL1 + 4, START, 4);
    }
}

static void tmpsns_init(Object *obj)
{
    IMX95Tmpsns *s = IMX95_TMPSNS(obj);
    s->temperature = 25000;
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, sample, s);
    memory_region_init_io(&s->iomem, obj, &tmpsns_ops, s, TYPE_IMX95_TMPSNS, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    object_property_add(obj, "temperature", "int", temperature_get,
                        temperature_set, NULL, NULL);
}

static void tmpsns_finalize(Object *obj)
{
    timer_del(&IMX95_TMPSNS(obj)->timer);
}

static const VMStateDescription vmstate_tmpsns = {
    .name = TYPE_IMX95_TMPSNS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IMX95Tmpsns, REG_COUNT),
        VMSTATE_INT64(temperature, IMX95Tmpsns),
        VMSTATE_BOOL(running, IMX95Tmpsns),
        VMSTATE_TIMER(timer, IMX95Tmpsns),
        VMSTATE_END_OF_LIST()
    },
};

static const Property tmpsns_properties[] = {
    DEFINE_PROP_BOOL("boot-enabled", IMX95Tmpsns, boot_enabled, false),
};

static void tmpsns_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->desc = "i.MX95 polling temperature sensor (no trip IRQ model)";
    dc->vmsd = &vmstate_tmpsns;
    device_class_set_props(dc, tmpsns_properties);
    device_class_set_legacy_reset(dc, tmpsns_reset);
}

static const TypeInfo tmpsns_info = {
    .name = TYPE_IMX95_TMPSNS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMX95Tmpsns),
    .instance_init = tmpsns_init,
    .instance_finalize = tmpsns_finalize,
    .class_init = tmpsns_class_init,
};

static void tmpsns_register_types(void)
{
    type_register_static(&tmpsns_info);
}
type_init(tmpsns_register_types)
