/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Local i.MX95 LPTMR time-counter model for the M7 Zephyr system clock.
 * Register contract: NXP LPTMR CSR/PSR/CMR/CNR, as used by Zephyr's
 * mcux_lptmr_timer.c and its MCUX HAL. Only PCS=2 (32.768 kHz) is wired;
 * external pulse counting and other clock sources are not implemented.
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"

#define TYPE_IMX95_LPTMR "imx95.lptmr"
OBJECT_DECLARE_SIMPLE_TYPE(IMX95LptmrState, IMX95_LPTMR)
#define TEN  (1u << 0)
#define TMS  (1u << 1)
#define TFC  (1u << 2)
#define TIE  (1u << 6)
#define TCF  (1u << 7)

struct IMX95LptmrState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer timer;
    uint32_t csr, psr, cmr, latched;
    int64_t epoch;
};

static uint32_t lptmr_divisor(IMX95LptmrState *s)
{
    if ((s->psr & 3) != 2 || (s->csr & TMS)) {
        return 0;
    }
    return (s->psr & 4) ? 1 : 1u << (((s->psr >> 3) & 15) + 1);
}

static uint64_t lptmr_ticks(IMX95LptmrState *s)
{
    uint32_t divisor = lptmr_divisor(s);
    if (!(s->csr & TEN) || !divisor) {
        return 0;
    }
    return muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->epoch,
                    32768, NANOSECONDS_PER_SECOND) / divisor;
}

static uint32_t lptmr_count(IMX95LptmrState *s)
{
    uint64_t count = lptmr_ticks(s);
    return (s->csr & TFC) ? count : count % ((uint64_t)s->cmr + 1);
}

static void lptmr_update(IMX95LptmrState *s)
{
    uint32_t divisor = lptmr_divisor(s);
    uint64_t elapsed, period, target;

    qemu_set_irq(s->irq, (s->csr & (TIE | TCF)) == (TIE | TCF));
    timer_del(&s->timer);
    if (!(s->csr & TEN) || (s->csr & TCF) || !divisor) {
        return;
    }
    elapsed = lptmr_ticks(s);
    period = (s->csr & TFC) ? (1ULL << 32) : (uint64_t)s->cmr + 1;
    target = (elapsed / period) * period + (uint64_t)s->cmr + 1;
    if (target <= elapsed) {
        target += period;
    }
    timer_mod(&s->timer, s->epoch +
              muldiv64(target * divisor, NANOSECONDS_PER_SECOND, 32768) +
              ((target * divisor) % 64 ? 1 : 0));
}

static void lptmr_expire(void *opaque)
{
    IMX95LptmrState *s = opaque;
    s->csr |= TCF;
    lptmr_update(s);
}

static uint64_t lptmr_read(void *opaque, hwaddr off, unsigned size)
{
    IMX95LptmrState *s = opaque;
    switch (off) {
    case 0: return s->csr;
    case 4: return s->psr;
    case 8: return s->cmr;
    case 12: return s->latched;
    default: return 0;
    }
}

static void lptmr_write(void *opaque, hwaddr off, uint64_t value, unsigned size)
{
    IMX95LptmrState *s = opaque;
    uint32_t old = s->csr;

    switch (off) {
    case 0:
        s->csr = (value & 0x7f) | (old & TCF & ~value);
        if (!(s->csr & TEN)) {
            s->csr &= ~TCF;
            s->latched = 0;
        }
        if (!(old & TEN) || !(s->csr & TEN)) {
            s->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        if ((s->csr & TEN) && !lptmr_divisor(s)) {
            qemu_log_mask(LOG_UNIMP, "LPTMR: unsupported clock/pulse mode\n");
        }
        break;
    case 4:
        if (!(s->csr & TEN)) {
            s->psr = value & 0x7f;
        }
        break;
    case 8: s->cmr = value; break;
    case 12: s->latched = lptmr_count(s); return;
    default: return;
    }
    lptmr_update(s);
}

static const MemoryRegionOps lptmr_ops = {
    .read = lptmr_read, .write = lptmr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void lptmr_reset(DeviceState *dev)
{
    IMX95LptmrState *s = IMX95_LPTMR(dev);
    s->csr = s->psr = s->cmr = s->latched = 0;
    s->epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    lptmr_update(s);
}

static int lptmr_post_load(void *opaque, int version_id)
{
    lptmr_update(opaque);
    return 0;
}

static const VMStateDescription vmstate_lptmr = {
    .name = TYPE_IMX95_LPTMR,
    .version_id = 1, .minimum_version_id = 1,
    .post_load = lptmr_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(csr, IMX95LptmrState),
        VMSTATE_UINT32(psr, IMX95LptmrState),
        VMSTATE_UINT32(cmr, IMX95LptmrState),
        VMSTATE_UINT32(latched, IMX95LptmrState),
        VMSTATE_INT64(epoch, IMX95LptmrState),
        VMSTATE_END_OF_LIST()
    },
};

static void lptmr_init(Object *obj)
{
    IMX95LptmrState *s = IMX95_LPTMR(obj);
    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, lptmr_expire, s);
    memory_region_init_io(&s->iomem, obj, &lptmr_ops, s,
                          TYPE_IMX95_LPTMR, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void lptmr_finalize(Object *obj)
{
    timer_del(&IMX95_LPTMR(obj)->timer);
}

static void lptmr_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->vmsd = &vmstate_lptmr;
    dc->desc = "i.MX95 LPTMR 32-bit time counter";
    device_class_set_legacy_reset(dc, lptmr_reset);
}

static const TypeInfo lptmr_type = {
    .name = TYPE_IMX95_LPTMR, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMX95LptmrState),
    .instance_init = lptmr_init, .instance_finalize = lptmr_finalize,
    .class_init = lptmr_class_init,
};
static void lptmr_register(void) { type_register_static(&lptmr_type); }
type_init(lptmr_register)
