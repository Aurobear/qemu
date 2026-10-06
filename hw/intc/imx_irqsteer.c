/*
 * NXP i.MX IRQSTEER interrupt multiplexer ("fsl,imx-irqsteer")
 *
 * Copyright (c) 2026, Kyle Fox
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The IRQSTEER funnels a large bank of device interrupt inputs onto a small
 * number of parent (GIC) outputs. Inputs are grouped 64 per output; within the
 * register file they are addressed by 32-bit registers with a REVERSED index
 * (register idx = reg_num - input/32 - 1), matching the Linux irq-imx-irqsteer
 * driver. Each input has a mask bit (CHANMASK, 1 = enabled) and a software-set
 * bit (CHANSET); the read-only CHANSTATUS exposes (raw_input | sw_set) & mask,
 * and a parent output asserts whenever any of its group's CHANSTATUS bits are
 * set. The i.MX95 displaymix instance has 512 inputs (16 registers) and 8
 * outputs; the DPU's frame-complete / shadow-load interrupts ride inputs in
 * group 1 (inputs 64..127 -> output 1).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_IMX_IRQSTEER "imx.irqsteer"
OBJECT_DECLARE_SIMPLE_TYPE(IMXIRQSteerState, IMX_IRQSTEER)

#define IRQSTEER_NUM_INPUTS   640
#define IRQSTEER_REG_NUM      (IRQSTEER_NUM_INPUTS / 32)        /* maximum: 20 */
#define IRQSTEER_NUM_OUTPUTS  ((IRQSTEER_NUM_INPUTS + 63) / 64) /* maximum: 10 */
#define IRQSTEER_MMIO_SIZE    0x1000

/* Bank offsets depend on the instance input count. */
#define IRQSTEER_CHANCTRL     0x00
#define IRQSTEER_CHANMASK0    0x04   /* bank 0 */
#define IRQSTEER_CHANSET0 (4 + s->num_inputs / 8)
#define IRQSTEER_CHANSTATUS0 (4 + s->num_inputs / 4)
#define IRQSTEER_MINTDIS (4 + 3 * s->num_inputs / 8)
#define IRQSTEER_MASTRSTAT (8 + 3 * s->num_inputs / 8)

struct IMXIRQSteerState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t num_inputs;
    uint32_t mintdis;
    uint32_t chanctrl;
    uint32_t mask[IRQSTEER_REG_NUM];
    uint32_t set[IRQSTEER_REG_NUM];     /* software-set interrupts */
    uint32_t inlevel[IRQSTEER_REG_NUM]; /* raw input line levels */

    qemu_irq out[IRQSTEER_NUM_OUTPUTS];
};

/* Register index for input line L: reversed, per the HW/driver convention. */
static inline int irqsteer_reg_index(IMXIRQSteerState *s, int line)
{
    return s->num_inputs / 32 - line / 32 - 1;
}

static inline uint32_t irqsteer_status(IMXIRQSteerState *s, int idx)
{
    return (s->inlevel[idx] | s->set[idx]) & s->mask[idx];
}

/* Recompute the parent outputs: output g ORs its group's two registers. */
static void irqsteer_update(IMXIRQSteerState *s)
{
    int g;

    for (g = 0; g < s->num_inputs / 64; g++) {
        /* Output g covers inputs [g*64, g*64+63] -> registers idx and idx-1. */
        int idx = irqsteer_reg_index(s, g * 64);
        uint32_t pending = irqsteer_status(s, idx);

        if (idx - 1 >= 0) {
            pending |= irqsteer_status(s, idx - 1);
        }
        qemu_set_irq(s->out[g], pending != 0 && !(s->mintdis & (1u << g)));
    }
}

/* GPIO input: an upstream device drives one of the IRQSTEER input lines. */
static void irqsteer_set_input(void *opaque, int line, int level)
{
    IMXIRQSteerState *s = opaque;
    int idx = irqsteer_reg_index(s, line);
    uint32_t bit = 1u << (line % 32);

    if (level) {
        s->inlevel[idx] |= bit;
    } else {
        s->inlevel[idx] &= ~bit;
    }
    irqsteer_update(s);
}

static uint64_t irqsteer_read(void *opaque, hwaddr off, unsigned size)
{
    IMXIRQSteerState *s = opaque;

    if (off == IRQSTEER_CHANCTRL) {
        return s->chanctrl;
    } else if (off >= IRQSTEER_CHANMASK0 && off < IRQSTEER_CHANSET0) {
        return s->mask[(off - IRQSTEER_CHANMASK0) / 4];
    } else if (off >= IRQSTEER_CHANSET0 && off < IRQSTEER_CHANSTATUS0) {
        return s->set[(off - IRQSTEER_CHANSET0) / 4];
    } else if (off >= IRQSTEER_CHANSTATUS0 && off < IRQSTEER_MINTDIS) {
        return irqsteer_status(s, (off - IRQSTEER_CHANSTATUS0) / 4);
    }
    if (off == IRQSTEER_MINTDIS) {
        return s->mintdis;
    }
    /* Other summary registers are not yet modelled. */
    return 0;
}

static void irqsteer_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    IMXIRQSteerState *s = opaque;

    if (off == IRQSTEER_CHANCTRL) {
        s->chanctrl = val;
    } else if (off >= IRQSTEER_CHANMASK0 && off < IRQSTEER_CHANSET0) {
        s->mask[(off - IRQSTEER_CHANMASK0) / 4] = val;
        irqsteer_update(s);
    } else if (off >= IRQSTEER_CHANSET0 && off < IRQSTEER_CHANSTATUS0) {
        s->set[(off - IRQSTEER_CHANSET0) / 4] = val;
        irqsteer_update(s);
    }
    if (off == IRQSTEER_MINTDIS) {
        s->mintdis = val;
        irqsteer_update(s);
    }
    /* CHANSTATUS is read-only. */
}

static const MemoryRegionOps irqsteer_ops = {
    .read = irqsteer_read,
    .write = irqsteer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void irqsteer_reset(DeviceState *dev)
{
    IMXIRQSteerState *s = IMX_IRQSTEER(dev);
    int i;

    s->chanctrl = 0;
    s->mintdis = 0;
    for (i = 0; i < IRQSTEER_REG_NUM; i++) {
        s->mask[i] = 0;
        s->set[i] = 0;
        s->inlevel[i] = 0;
    }
    irqsteer_update(s);
}

static void irqsteer_init(Object *obj)
{
    IMXIRQSteerState *s = IMX_IRQSTEER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &irqsteer_ops, s,
                          TYPE_IMX_IRQSTEER, IRQSTEER_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void irqsteer_realize(DeviceState *dev, Error **errp)
{
    IMXIRQSteerState *s = IMX_IRQSTEER(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (s->num_inputs != 512 && s->num_inputs != 640) {
        error_setg(errp, "IRQSTEER num-inputs must be 512 or 640");
        return;
    }
    qdev_init_gpio_in(dev, irqsteer_set_input, s->num_inputs);
    for (int i = 0; i < s->num_inputs / 64; i++) {
        sysbus_init_irq(sbd, &s->out[i]);
    }
}

static int irqsteer_post_load(void *opaque, int version_id)
{
    irqsteer_update(opaque);
    return 0;
}

static const VMStateDescription vmstate_irqsteer = {
    .name = TYPE_IMX_IRQSTEER,
    .post_load = irqsteer_post_load,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_EQUAL(num_inputs, IMXIRQSteerState),
        VMSTATE_UINT32(mintdis, IMXIRQSteerState),
        VMSTATE_UINT32(chanctrl, IMXIRQSteerState),
        VMSTATE_UINT32_ARRAY(mask, IMXIRQSteerState, IRQSTEER_REG_NUM),
        VMSTATE_UINT32_ARRAY(set, IMXIRQSteerState, IRQSTEER_REG_NUM),
        VMSTATE_UINT32_ARRAY(inlevel, IMXIRQSteerState, IRQSTEER_REG_NUM),
        VMSTATE_END_OF_LIST()
    },
};

static const Property irqsteer_properties[] = {
    DEFINE_PROP_UINT32("num-inputs", IMXIRQSteerState, num_inputs, 512),
};

static void irqsteer_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = irqsteer_realize;
    device_class_set_props(dc, irqsteer_properties);
    dc->vmsd = &vmstate_irqsteer;
    device_class_set_legacy_reset(dc, irqsteer_reset);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "NXP i.MX IRQSTEER";
}

static const TypeInfo imx_irqsteer_info = {
    .name          = TYPE_IMX_IRQSTEER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMXIRQSteerState),
    .instance_init = irqsteer_init,
    .class_init    = irqsteer_class_init,
};

static void imx_irqsteer_register_types(void)
{
    type_register_static(&imx_irqsteer_info);
}

type_init(imx_irqsteer_register_types)
