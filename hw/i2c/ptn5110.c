/*
 * PTN5110 TCPCI subset for local FRDM-i.MX95 functional experiments.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Register definitions: NXP PTN5110 rev 1.6 table 7, AN12137 rev 1.0
 * section 8. No BMC/USB data path, PD peer, FRS, analogue protection or
 * physical timing. The optional partner is a non-PD 5 V source on CC1.
 * Unsupported PD transmissions fail; they never synthesize GoodCRC.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_PTN5110 "ptn5110"
OBJECT_DECLARE_SIMPLE_TYPE(PTN5110State, PTN5110)

#define ALERT       0x10
#define ALERT_MASK  0x12
#define POWER_MASK  0x14
#define FAULT_MASK  0x15
#define EXT_MASK    0x16
#define TCPC_CTRL   0x19
#define ROLE_CTRL   0x1a
#define POWER_CTRL  0x1c
#define CC_STATUS   0x1d
#define POWER_STATUS 0x1e
#define FAULT_STATUS 0x1f
#define EXT_STATUS  0x20
#define COMMAND     0x23
#define TRANSMIT    0x50
#define VBUS_VOLTAGE 0x70
#define ALERT_CC    (1u << 0)
#define ALERT_POWER (1u << 1)
#define ALERT_TX_FAILED (1u << 4)
#define ALERT_FAULT (1u << 9)
#define ALERT_EXT   (1u << 13)

struct PTN5110State {
    I2CSlave parent_obj;
    qemu_irq alert_n;
    uint8_t regs[256];
    uint8_t pointer;
    bool address_phase;
    bool partner_source;
    bool looking;
    bool resolved_sink;
    bool sink_enabled;
    bool source_enabled;
    bool vbus_detect;
};

static uint16_t ptn5110_word(PTN5110State *s, unsigned reg)
{
    return s->regs[reg] | (s->regs[reg + 1] << 8);
}

static void ptn5110_set_word(PTN5110State *s, unsigned reg, uint16_t value)
{
    s->regs[reg] = value;
    s->regs[reg + 1] = value >> 8;
}

static void ptn5110_alert(PTN5110State *s, uint16_t bits)
{
    ptn5110_set_word(s, ALERT, ptn5110_word(s, ALERT) | bits);
}

static void ptn5110_update_irq(PTN5110State *s)
{
    if (s->regs[FAULT_STATUS] & s->regs[FAULT_MASK]) {
        ptn5110_alert(s, ALERT_FAULT);
    }
    qemu_set_irq(s->alert_n,
                 !(ptn5110_word(s, ALERT) & ptn5110_word(s, ALERT_MASK)));
}

static void ptn5110_update_port(PTN5110State *s)
{
    uint8_t role = s->regs[ROLE_CTRL];
    bool drp = role & 0x40;
    bool rd = (role & 3) == 2;
    bool attached = s->partner_source &&
                    (rd || (drp && (s->looking || s->resolved_sink)));
    uint8_t cc;
    bool vbus = attached || s->source_enabled;
    uint8_t power = (s->vbus_detect ? 8 : 0) |
                    (s->vbus_detect && vbus ? 4 : 0) |
                    (s->sink_enabled && attached ? 1 : 0) |
                    (s->source_enabled ? 0x10 : 0) |
                    (s->regs[POWER_CTRL] & 1 ? 2 : 0);
    uint8_t ext = s->vbus_detect && !vbus ? 1 : 0;
    uint8_t cc_change;

    if (attached && s->looking) {
        s->looking = false;
        s->resolved_sink = true;
    }
    cc = attached ? 0x11 : (s->looking ? 0x20 :
                           (s->resolved_sink ? 0x10 : 0));
    cc_change = s->regs[CC_STATUS] ^ cc;

    /* Looking4Connection alone is only an alert when explicitly enabled. */
    if ((cc_change & ~0x20) ||
        (cc_change && (s->regs[TCPC_CTRL] & 0x40))) {
        ptn5110_alert(s, ALERT_CC);
    }
    if ((s->regs[POWER_STATUS] ^ power) & s->regs[POWER_MASK]) {
        ptn5110_alert(s, ALERT_POWER);
    }
    if ((s->regs[EXT_STATUS] ^ ext) & s->regs[EXT_MASK]) {
        ptn5110_alert(s, ALERT_EXT);
    }
    s->regs[CC_STATUS] = cc;
    s->regs[POWER_STATUS] = power;
    s->regs[EXT_STATUS] = ext;
    ptn5110_set_word(s, VBUS_VOLTAGE, vbus ? 5000 / 25 : 0);
    trace_ptn5110_port(s->partner_source, cc, power);
    ptn5110_update_irq(s);
}

static int ptn5110_command(PTN5110State *s, uint8_t command)
{
    switch (command) {
    case 0x11: /* WakeI2C: the functional model does not sleep. */
    case 0xff: /* I2CIdle */
    case 0xaa: /* RxOneMore: no PD partner, so no receive completion. */
        break;
    case 0x22:
        s->vbus_detect = false;
        break;
    case 0x33:
        s->vbus_detect = true;
        break;
    case 0x44:
        s->sink_enabled = false;
        break;
    case 0x55:
        s->sink_enabled = true;
        break;
    case 0x66:
        s->source_enabled = false;
        break;
    case 0x77:
        s->source_enabled = true;
        break;
    case 0x99:
        s->looking = true;
        s->resolved_sink = false;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "ptn5110: unsupported command 0x%02x\n",
                      command);
        return -1;
    }
    ptn5110_update_port(s);
    return 0;
}

static int ptn5110_send(I2CSlave *i2c, uint8_t data)
{
    PTN5110State *s = PTN5110(i2c);
    uint8_t reg;

    if (s->address_phase) {
        s->pointer = data;
        s->address_phase = false;
        return 0;
    }
    reg = s->pointer++;
    trace_ptn5110_write(reg, data);
    switch (reg) {
    case ALERT:
    case ALERT + 1:
    case FAULT_STATUS:
    case 0x21: /* ALERT_EXTENDED */
        s->regs[reg] &= ~data;
        break;
    case COMMAND:
        return ptn5110_command(s, data);
    case TRANSMIT:
        s->regs[reg] = data & 0x37;
        /* No GoodCRC-capable partner is modelled, including when attached. */
        ptn5110_alert(s, ALERT_TX_FAILED);
        break;
    case ROLE_CTRL:
        s->regs[reg] = data & 0x7f;
        s->looking = false;
        s->resolved_sink = false;
        ptn5110_update_port(s);
        break;
    case POWER_CTRL:
        s->regs[reg] = data;
        ptn5110_update_port(s);
        break;
    case ALERT_MASK:
    case ALERT_MASK + 1:
    case POWER_MASK:
    case FAULT_MASK:
    case EXT_MASK:
    case 0x17: /* ALERT_EXTENDED_MASK */
    case 0x18: /* CONFIG_STANDARD_OUTPUT */
    case TCPC_CTRL:
    case 0x1b: /* FAULT_CONTROL */
    case 0x2e: /* MESSAGE_HEADER_INFO */
    case 0x2f: /* RECEIVE_DETECT */
    case 0x51 ... 0x6f: /* TX byte count, header, payload */
    case 0x72 ... 0x79: /* Voltage thresholds; analogue alarms not modelled. */
        s->regs[reg] = data;
        break;
    case 0x00 ... 0x0b: /* Identification */
    case CC_STATUS:
    case POWER_STATUS:
    case EXT_STATUS:
    case 0x24 ... 0x29: /* Capabilities */
    case 0x30 ... 0x4f: /* Empty RX buffer */
    case VBUS_VOLTAGE:
    case VBUS_VOLTAGE + 1:
        break; /* Read-only. */
    default:
        qemu_log_mask(LOG_UNIMP, "ptn5110: unimplemented write 0x%02x\n", reg);
        break;
    }
    ptn5110_update_irq(s);
    return 0;
}

static uint8_t ptn5110_recv(I2CSlave *i2c)
{
    PTN5110State *s = PTN5110(i2c);
    uint8_t reg = s->pointer++;

    trace_ptn5110_read(reg, s->regs[reg]);
    return s->regs[reg];
}

static int ptn5110_event(I2CSlave *i2c, enum i2c_event event)
{
    PTN5110State *s = PTN5110(i2c);

    if (event == I2C_START_SEND) {
        s->address_phase = true;
    }
    return 0;
}

static void ptn5110_reset(DeviceState *dev)
{
    PTN5110State *s = PTN5110(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = 0;
    s->address_phase = true;
    s->looking = s->sink_enabled = s->source_enabled = false;
    s->resolved_sink = false;
    s->vbus_detect = true;
    /* Reset completion is instantaneous; no analogue/MTP startup timing. */
    ptn5110_set_word(s, 0x00, 0x1fc9);
    ptn5110_set_word(s, 0x02, 0x5110);
    ptn5110_set_word(s, 0x04, 0x0001);
    ptn5110_set_word(s, 0x06, 0x0012);
    ptn5110_set_word(s, 0x08, 0x3011);
    ptn5110_set_word(s, 0x0a, 0x2010);
    ptn5110_set_word(s, ALERT_MASK, 0x7fff);
    s->regs[POWER_MASK] = s->regs[FAULT_MASK] = 0xff;
    s->regs[0x18] = 0x60;
    s->regs[ROLE_CTRL] = 0x0a; /* Board experiment starts with Rd on both CCs. */
    s->regs[POWER_CTRL] = 0x60;
    s->regs[FAULT_STATUS] = 0x80;
    ptn5110_set_word(s, 0x24, 0x7edf);
    ptn5110_set_word(s, 0x26, 0x37c7);
    s->regs[0x28] = 0x06;
    s->regs[0x29] = 0x40;
    s->regs[0x2e] = 0x02;
    ptn5110_set_word(s, 0x72, 0x008c);
    ptn5110_set_word(s, 0x74, 0x0020);
    ptn5110_update_port(s);
}

static bool ptn5110_get_partner(Object *obj, Error **errp)
{
    return PTN5110(obj)->partner_source;
}

static void ptn5110_set_partner(Object *obj, bool connected, Error **errp)
{
    PTN5110State *s = PTN5110(obj);

    s->partner_source = connected;
    ptn5110_update_port(s);
}

static int ptn5110_post_load(void *opaque, int version_id)
{
    ptn5110_update_irq(opaque);
    return 0;
}

static const VMStateDescription vmstate_ptn5110 = {
    .name = TYPE_PTN5110,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ptn5110_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, PTN5110State),
        VMSTATE_UINT8_ARRAY(regs, PTN5110State, 256),
        VMSTATE_UINT8(pointer, PTN5110State),
        VMSTATE_BOOL(address_phase, PTN5110State),
        VMSTATE_BOOL(partner_source, PTN5110State),
        VMSTATE_BOOL(looking, PTN5110State),
        VMSTATE_BOOL(resolved_sink, PTN5110State),
        VMSTATE_BOOL(sink_enabled, PTN5110State),
        VMSTATE_BOOL(source_enabled, PTN5110State),
        VMSTATE_BOOL(vbus_detect, PTN5110State),
        VMSTATE_END_OF_LIST()
    },
};

static void ptn5110_init(Object *obj)
{
    PTN5110State *s = PTN5110(obj);

    qdev_init_gpio_out(DEVICE(obj), &s->alert_n, 1);
    object_property_add_bool(obj, "partner-source", ptn5110_get_partner,
                             ptn5110_set_partner);
    object_property_set_description(obj, "partner-source",
        "Experimental non-PD 5V source on CC1; no USB data connection");
}

static void ptn5110_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(klass);

    ic->event = ptn5110_event;
    ic->send = ptn5110_send;
    ic->recv = ptn5110_recv;
    device_class_set_legacy_reset(dc, ptn5110_reset);
    dc->vmsd = &vmstate_ptn5110;
    dc->desc = "PTN5110 TCPCI subset (no USB data/PD PHY)";
}

static const TypeInfo ptn5110_info = {
    .name = TYPE_PTN5110,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(PTN5110State),
    .instance_init = ptn5110_init,
    .class_init = ptn5110_class_init,
};

static void ptn5110_register_types(void)
{
    type_register_static(&ptn5110_info);
}
type_init(ptn5110_register_types)
