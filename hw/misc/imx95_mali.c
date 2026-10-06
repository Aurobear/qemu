/*
 * NXP i.MX 95 Mali GPU — identify-tier model (Linux sees it, fails honestly)
 *
 * Copyright (c) 2026, Kyle Fox
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Local GPU identification and GPU interrupt-register model. Linux panthor
 * identifies the G310. Full CSF support and rendering remain absent.
 * The opt-in mmu-experimental property adds a staged 4K LPAE data path.
 * It is not enabled by default or a complete GPU implementation.
 * Default capability registers stay zero: do not advertise hardware which is not
 * modelled just to advance probe. GPU commands, including reset and cache
 * flush, do not fabricate completion events.
 *
 * Register offsets/event bits follow Linux panthor_gpu_regs.h; interrupt
 * RAWSTAT is software-set, CLEAR is W1C, STATUS is RAWSTAT & MASK. Only the
 * GPU-control bank is always available; staged MMU interrupts require opt-in.
 * mcu-experimental adds Cortex-M7 execution up to unsupported internal MMIO.
 * Job completion and rendering remain absent.
 *
 * GPU_ID (new "ID2" format, uapi mali_kbase_gpu_id.h):
 *   arch_major[31:28] arch_minor[27:24] arch_rev[23:20] product_major[19:16]
 *   version_major[15:12] version_minor[11:4] version_status[3:0]
 * Mali-G310 = product model TVAX = MODEL_MAKE(arch_major=10, product_major=4)
 * (the G310/G510 share the TVAX product code, differing only in core count).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/boards.h"
#include "hw/arm/armv7m.h"
#include "hw/core/clock.h"
#include "hw/core/qdev-clock.h"
#include "system/cpus.h"
#include "target/arm/arm-powerctl.h"
#include "target/arm/cpu.h"
#include "target/arm/internals.h"
#include "migration/blocker.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "migration/vmstate.h"
#include "trace.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_IMX95_MALI "imx95.mali"
OBJECT_DECLARE_SIMPLE_TYPE(IMX95MaliState, IMX95_MALI)

/* DT reg window for gpu@4d900000 (the driver ioremaps the whole thing). */
#define MALI_MMIO_SIZE   0x480000

/* GPU control register offsets the kbase probe touches. */
#define MALI_GPU_ID      0x0
#define MALI_IRQ_RAW     0x20
#define MALI_IRQ_CLEAR   0x24
#define MALI_IRQ_MASK    0x28
#define MALI_IRQ_STATUS  0x2c

/* GPU events understood by the frozen panthor driver, not capability bits. */
#define MALI_IRQ_BITS    0x000e0703u

/*
 * Mali-G310 (Valhall, CSF) GPU_ID = product TVAX = arch_major=10,
 * arch_minor=12, arch_rev=7, product_major=4, version_status=1. Read back
 * from the GPU_ID register of a real i.MX 95 (FRDM-IMX95-PRO), where kbase
 * reports "GPU identified as 0x4 arch 10.12.7 r0p0".
 * (arch_minor MUST be 12: kbase builds its register-map LUT from
 * arch_major.minor.rev, and a wrong minor selects an incomplete map -> gpuprops
 * dereferences a NULL regmap entry and oopses.)
 */
#define MALI_GPU_ID_G310 0xAC740001u

#define MALI_AS_COUNT 16
#define MALI_IOMMU_INDEX_COUNT 2
#define MALI_VA_BITS 48
#define MALI_PA_BITS 40
#define MALI_PA_MASK ((1ULL << MALI_PA_BITS) - 1)
#define TYPE_IMX95_MALI_IOMMU "imx95-mali-iommu"

typedef struct IMX95MaliAS {
    IOMMUMemoryRegion iommu;
    AddressSpace dma_as;
    IMX95MaliState *gpu;
    unsigned number;
    uint64_t transtab, transcfg, memattr, lockaddr;
    uint64_t active_transtab, active_transcfg, active_memattr;
    uint64_t faultaddr;
    uint32_t faultstatus;
    bool locked;
} IMX95MaliAS;

struct IMX95MaliState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t gpu_id;
    uint32_t irq_raw;
    uint32_t irq_mask;
    qemu_irq irq;
    bool mmu_experimental;
    uint32_t mmu_raw, mmu_mask;
    qemu_irq mmu_irq;
    IMX95MaliAS as[MALI_AS_COUNT];
    bool mcu_experimental;
    ARMv7MState mcu;
    Clock *mcu_clock;
    MemoryRegion mcu_memory, mcu_window, mcu_mmio;
    uint32_t mcu_control, mcu_status;
    /* Only the independently verified lower 64 MiB of window 1. */
    MemoryRegion mcu_map_memory, mcu_map_alias[MALI_AS_COUNT];
    uint32_t mcu_map_low, mcu_map_high, mcu_map_as;
    Error *mcu_migration_blocker;
};

static void imx95_mali_update_irq(IMX95MaliState *s)
{
    uint32_t status = s->irq_raw & s->irq_mask;

    trace_imx95_mali_irq(s->irq_raw, s->irq_mask, status);
    qemu_set_irq(s->irq, status != 0);
}

/* Local staged MMU experiment. No MCU, shader or CSF completion is implied. */
static void imx95_mali_mmu_irq(IMX95MaliState *s)
{
    qemu_set_irq(s->mmu_irq, !!(s->mmu_raw & s->mmu_mask));
}

static void imx95_mali_invalidate(IMX95MaliAS *as)
{
    IOMMUTLBEvent event = {
        .type = IOMMU_NOTIFIER_UNMAP,
        .entry = {
            .target_as = &address_space_memory,
            .addr_mask = (1ULL << MALI_VA_BITS) - 1,
            .perm = IOMMU_NONE,
        },
    };

    for (int idx = 0; idx < MALI_IOMMU_INDEX_COUNT; idx++) {
        memory_region_notify_iommu(&as->iommu, idx, event);
    }
}

static void imx95_mali_mcu_fail_stop(IMX95MaliState *s);

static void imx95_mali_fault(IMX95MaliAS *as, hwaddr addr,
                             IOMMUAccessFlags flags, uint32_t exception)
{
    IMX95MaliState *s = as->gpu;
    unsigned access = flags & IOMMU_EXEC ? 1 : flags & IOMMU_WO ? 3 : 2;

    /* Preserve the first fault until software acknowledges this AS IRQ. */
    if (!(s->mmu_raw & (1U << as->number))) {
        as->faultaddr = addr;
        as->faultstatus = exception | (access << 8);
        s->mmu_raw |= 1U << as->number;
    }
    trace_imx95_mali_mmu_fault(as->number, addr, as->faultstatus);
    imx95_mali_mmu_irq(s);
    if (s->mcu_experimental && as->number == 0 && s->mcu_status == 1) {
        imx95_mali_mcu_fail_stop(s);
    }
}

static IOMMUTLBEntry imx95_mali_translate(IOMMUMemoryRegion *iommu,
                                          hwaddr addr, IOMMUAccessFlags flags,
                                          int iommu_idx)
{
    BQL_LOCK_GUARD(); /* CPU walks share AS state and fault IRQs with MMIO. */
    IMX95MaliAS *as = container_of(iommu, IMX95MaliAS, iommu);
    IOMMUTLBEntry entry = {
        .target_as = &address_space_memory,
        .iova = addr & ~0xfffULL,
        .addr_mask = 0xfff,
        .perm = IOMMU_NONE,
    };
    uint64_t cfg = as->active_transcfg, table = as->active_transtab;
    unsigned bits = 55 - extract64(cfg, 6, 6);
    unsigned level, shift;
    uint32_t fault = 0xc0;
    bool readonly = false, xn = false, privileged_only = false;
    bool privileged = (flags & IOMMU_PRIV) || iommu_idx == 1;

    if (!as->gpu->mmu_experimental) {
        return entry;
    }
    /* Only the AArch64 4K format used by the frozen panthor is enabled.
     * Other modes fail closed, rather than aliasing arbitrary host RAM.
     */
    if ((cfg & 0xf) != 6 || bits < 16 || bits > MALI_VA_BITS ||
        (cfg & (1ULL << 22))) {
        goto fault;
    }
    if (addr >> bits) {
        fault = 0xe0;
        goto fault;
    }
    /* No concurrent GPU engine exists yet. Locked AS accesses fail closed;
     * this is not a model of a hardware-stalled CSF/MCU transaction.
     */
    if (as->locked) {
        return entry;
    }
    level = 4 - DIV_ROUND_UP(bits - 12, 9);
    table &= ~0xfffULL;
    for (; level < 4; level++) {
        MemTxResult result;
        uint64_t pte, pa, mask;
        unsigned kind;
        IOMMUAccessFlags perm = IOMMU_RW | IOMMU_EXEC;

        shift = 12 + 9 * (3 - level);
        if (table & ~MALI_PA_MASK) {
            fault = 0xe4 + level;
            goto fault;
        }
        pte = address_space_ldq_le(&address_space_memory,
                                  table + (((addr >> shift) & 511) * 8),
                                  MEMTXATTRS_UNSPECIFIED, &result);
        if (result != MEMTX_OK) {
            fault = 0xc0 + level;
            goto fault;
        }
        kind = pte & 3;
        if (kind == 3 && level < 3) {
            if (!(cfg & (1ULL << 33))) {
                readonly |= !!(pte & (1ULL << 62));
                privileged_only |= !!(pte & (1ULL << 61));
                xn |= !!(pte & (1ULL << (privileged ? 59 : 60)));
            }
            table = pte & 0x0000fffffffff000ULL;
            continue;
        }
        if (!((kind == 3 && level == 3) ||
              (kind == 1 && (level == 1 || level == 2)))) {
            fault = 0xc0 + level;
            goto fault;
        }
        mask = (1ULL << shift) - 1;
        pa = pte & 0x0000fffffffff000ULL;
        if ((pa & ~MALI_PA_MASK) || (pa & mask)) {
            fault = 0xe4 + level;
            goto fault;
        }
        if (!(pte & (1ULL << 10)) && !(cfg & (1ULL << 34))) {
            fault = 0xd8 + level;
            goto fault;
        }
        /* AP[2], hierarchical read-only, XN and WXN. No HTTU/dirty updates. */
        if (!privileged && (privileged_only || !(pte & (1ULL << 6)))) {
            fault = 0xc8 + level;
            goto fault;
        }
        readonly |= !!(pte & (1ULL << 7));
        if (readonly) {
            perm &= ~IOMMU_WO;
        }
        if (xn || (pte & (1ULL << (privileged ? 53 : 54))) ||
            ((cfg & (1ULL << 35)) && !readonly)) {
            perm &= ~IOMMU_EXEC;
        }
        if ((flags & (IOMMU_RW | IOMMU_EXEC)) & ~perm) {
            fault = 0xc8 + level;
            goto fault;
        }
        if (((as->active_memattr >> (extract64(pte, 2, 3) * 8)) & 0xc0) == 0xc0) {
            fault = 0xe8 + level;
            goto fault;
        }
        entry.iova = addr & ~mask;
        entry.addr_mask = mask;
        entry.translated_addr = pa;
        entry.perm = perm;
        trace_imx95_mali_mmu_translate(as->number, addr, pa | (addr & mask), perm);
        return entry;
    }
 fault:
    /* Permission-only queries must not invent a read fault before the CPU
     * reports the actual fetch/load/store that was rejected. */
    if (flags & (IOMMU_RW | IOMMU_EXEC)) {
        imx95_mali_fault(as, addr, flags, fault);
    }
    return entry;
}

static uint32_t imx95_mali_as_read(IMX95MaliAS *as, unsigned off)
{
    uint64_t value;

    switch (off & ~7) {
    case 0x00: value = as->transtab; break;
    case 0x08: value = as->memattr; break;
    case 0x10: value = as->lockaddr; break;
    case 0x18: return off == 0x1c ? as->faultstatus : 0;
    case 0x20: value = as->faultaddr; break;
    case 0x28: return 0; /* Synchronous command processing, no pending work. */
    case 0x30: value = as->transcfg; break;
    default: return 0;
    }
    return value >> ((off & 4) * 8);
}

static void imx95_mali_as_write(IMX95MaliAS *as, unsigned off, uint32_t value)
{
    uint64_t *reg;

    if (off == 0x18) {
        switch (value) {
        case 0: return;
        case 1:
            as->active_transtab = as->transtab;
            as->active_transcfg = as->transcfg;
            as->active_memattr = as->memattr;
            break;
        case 2: as->locked = true; break;
        case 3: as->locked = false; break;
        case 4: case 5:
            /* No private data/TLB cache or asynchronous writes in this stage. */
            as->locked = false;
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "imx95.mali: AS command %u not implemented\n", value);
            return;
        }
        imx95_mali_invalidate(as);
        return;
    }
    switch (off & ~7) {
    case 0x00: reg = &as->transtab; break;
    case 0x08: reg = &as->memattr; break;
    case 0x10: reg = &as->lockaddr; break;
    case 0x30: reg = &as->transcfg; break;
    default: return;
    }
    *reg = deposit64(*reg, (off & 4) * 8, 32, value);
}

/* Experimental mapping protocol corroborated by original firmware read/write
 * accessors and public CSF research. Low/high stage an aligned GPU VA; the AS
 * write applies the tuple atomically in this model. Hardware commit timing and
 * the upper half of the published 128 MiB window are deliberately not claimed.
 * Unknown control registers (including 0x22100) still fail closed.
 */
static void imx95_mali_mcu_map_reset(IMX95MaliState *s)
{
    if (!s->mmu_experimental) {
        return;
    }
    memory_region_transaction_begin();
    for (unsigned i = 0; i < MALI_AS_COUNT; i++) {
        memory_region_set_enabled(&s->mcu_map_alias[i], false);
    }
    memory_region_transaction_commit();
    s->mcu_map_low = s->mcu_map_high = s->mcu_map_as = 0;
}

static bool imx95_mali_mcu_map_write(IMX95MaliState *s, hwaddr off,
                                    uint32_t value)
{
    uint64_t base;

    if (!s->mmu_experimental) {
        return false;
    }
    switch (off) {
    case 0x22108:
        if (value & 0x03ffffff) {
            return false;
        }
        s->mcu_map_low = value;
        return true;
    case 0x2210c:
        if (value & 0xffff0000) {
            return false;
        }
        s->mcu_map_high = value;
        return true;
    case 0x22104:
        if (value >= MALI_AS_COUNT) {
            return false;
        }
        base = ((uint64_t)s->mcu_map_high << 32) | s->mcu_map_low;
        memory_region_transaction_begin();
        for (unsigned i = 0; i < MALI_AS_COUNT; i++) {
            memory_region_set_enabled(&s->mcu_map_alias[i], false);
        }
        memory_region_set_alias_offset(&s->mcu_map_alias[value], base);
        memory_region_set_enabled(&s->mcu_map_alias[value], true);
        memory_region_transaction_commit();
        s->mcu_map_as = value;
        trace_imx95_mali_mcu_map(value, base);
        return true;
    default:
        return false;
    }
}

/* This local MCU path deliberately stops at unmodelled internal registers.
 * It executes the original firmware; it never fabricates a firmware boot IRQ.
 */
static void imx95_mali_mcu_work(CPUState *cs, run_on_cpu_data data)
{
    IMX95MaliState *s = data.host_ptr;

    if (!s->mcu_control) {
        imx95_mali_mcu_map_reset(s);
        cs->halted = 1;
        arm_set_cpu_power_state(s->mcu.cpu, PSCI_OFF);
        memory_region_set_enabled(&s->mcu_window, false);
        s->mcu_status = 0;
    } else if (!s->mcu_status) {
        memory_region_set_enabled(&s->mcu_window, true);
        cpu_reset(cs);
        if (s->mmu_raw & 1) {
            s->mcu_status = 2; /* Halt: reset vector could not be translated. */
        } else {
            cs->halted = 0;
            arm_set_cpu_power_state(s->mcu.cpu, PSCI_ON);
            s->mcu_status = 1;
        }
    }
    trace_imx95_mali_mcu_state(s->mcu_control, s->mcu_status,
                              s->mcu.cpu->env.regs[15]);
}

static void imx95_mali_mcu_fail_stop(IMX95MaliState *s)
{
    /* Diagnostic fail-stop, not a hardware completion or successful MMIO.
     * Keep an unsupported firmware access from recursively faulting in its
     * exception handler and aborting the whole experimental machine.
     */
    s->mcu_status = 2;
    arm_set_cpu_power_state(s->mcu.cpu, PSCI_OFF);
    cpu_interrupt(CPU(s->mcu.cpu), CPU_INTERRUPT_HALT);
    trace_imx95_mali_mcu_state(s->mcu_control, s->mcu_status,
                              s->mcu.cpu->env.regs[15]);
}

static MemTxResult imx95_mali_mcu_read(void *opaque, hwaddr off,
                                      uint64_t *value, unsigned size,
                                      MemTxAttrs attrs)
{
    IMX95MaliState *s = opaque;

    if (size == 4) {
        switch (off) {
        case 0x22104: *value = s->mcu_map_as; return MEMTX_OK;
        case 0x22108: *value = s->mcu_map_low; return MEMTX_OK;
        case 0x2210c: *value = s->mcu_map_high; return MEMTX_OK;
        }
    }
    *value = 0;
    trace_imx95_mali_mcu_mmio(false, off + 0x40000000, 0, size,
                             s->mcu.cpu->env.regs[15]);
    imx95_mali_mcu_fail_stop(s);
    return MEMTX_ERROR;
}

static MemTxResult imx95_mali_mcu_write(void *opaque, hwaddr off,
                                       uint64_t value, unsigned size,
                                       MemTxAttrs attrs)
{
    IMX95MaliState *s = opaque;

    if (size == 4 && imx95_mali_mcu_map_write(s, off, value)) {
        return MEMTX_OK;
    }
    trace_imx95_mali_mcu_mmio(true, off + 0x40000000, value, size,
                             s->mcu.cpu->env.regs[15]);
    imx95_mali_mcu_fail_stop(s);
    return MEMTX_ERROR;
}

static const MemoryRegionOps imx95_mali_mcu_ops = {
    .read_with_attrs = imx95_mali_mcu_read,
    .write_with_attrs = imx95_mali_mcu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t imx95_mali_read(void *opaque, hwaddr off, unsigned size)
{
    IMX95MaliState *s = opaque;
    uint32_t value = 0;

    if (s->mmu_experimental && off >= 0x2400 && off < 0x2800) {
        value = imx95_mali_as_read(&s->as[(off - 0x2400) / 64], off & 63);
        goto done;
    }
    switch (off) {
    case 0x700: value = s->mcu_experimental ? s->mcu_control : 0; break;
    case 0x704: value = s->mcu_experimental ? s->mcu_status : 0; break;
    case 0x14:
        value = s->mmu_experimental ? (MALI_PA_BITS << 8) | MALI_VA_BITS : 0;
        break;
    case 0x18:
        value = s->mmu_experimental ? 0xffff : 0;
        break;
    case 0x2000: value = s->mmu_experimental ? s->mmu_raw : 0; break;
    case 0x2008: value = s->mmu_experimental ? s->mmu_mask : 0; break;
    case 0x200c: value = s->mmu_experimental ? s->mmu_raw & s->mmu_mask : 0; break;
    case MALI_GPU_ID:
        value = s->gpu_id;
        break;
    case MALI_IRQ_RAW:
        value = s->irq_raw;
        break;
    case MALI_IRQ_MASK:
        value = s->irq_mask;
        break;
    case MALI_IRQ_STATUS:
        value = s->irq_raw & s->irq_mask;
        break;
    default:
        break;
    }
 done:
    trace_imx95_mali_read(off, value, size);
    return value;
}

static void imx95_mali_write(void *opaque, hwaddr off, uint64_t val,
                             unsigned size)
{
    IMX95MaliState *s = opaque;

    trace_imx95_mali_write(off, val, size);
    if (s->mcu_experimental && off == 0x700) {
        if (val <= 2) {
            s->mcu_control = val;
            async_run_on_cpu(CPU(s->mcu.cpu), imx95_mali_mcu_work,
                             RUN_ON_CPU_HOST_PTR(s));
        }
        return;
    }
    if (s->mmu_experimental && off >= 0x2400 && off < 0x2800) {
        imx95_mali_as_write(&s->as[(off - 0x2400) / 64], off & 63, val);
        return;
    }
    if (s->mmu_experimental && off >= 0x2000 && off <= 0x200c) {
        switch (off) {
        case 0x2000: s->mmu_raw |= val & 0xffff; break;
        case 0x2004: s->mmu_raw &= ~val; break;
        case 0x2008: s->mmu_mask = val & 0xffff; break;
        default: return;
        }
        imx95_mali_mmu_irq(s);
        return;
    }
    switch (off) {
    case MALI_IRQ_RAW:
        s->irq_raw |= val & MALI_IRQ_BITS;
        break;
    case MALI_IRQ_CLEAR:
        s->irq_raw &= ~val;
        break;
    case MALI_IRQ_MASK:
        s->irq_mask = val & MALI_IRQ_BITS;
        break;
    default:
        /* No synthetic reset, power, cache, MMU or firmware completion. */
        return;
    }
    imx95_mali_update_irq(s);
}

static void imx95_mali_reset(DeviceState *dev)
{
    IMX95MaliState *s = IMX95_MALI(dev);

    for (unsigned i = 0; i < MALI_AS_COUNT; i++) {
        IMX95MaliAS *as = &s->as[i];

        as->transtab = as->memattr = as->lockaddr = 0;
        as->active_transtab = as->active_memattr = 0;
        as->transcfg = as->active_transcfg = 1; /* UNMAPPED */
        as->faultaddr = as->faultstatus = 0;
        as->locked = false;
        imx95_mali_invalidate(as);
    }
    imx95_mali_mcu_map_reset(s);
    if (s->mcu_experimental) {
        s->mcu_control = 0;
        memory_region_set_enabled(&s->mcu_window, false);
        async_run_on_cpu(CPU(s->mcu.cpu), imx95_mali_mcu_work,
                         RUN_ON_CPU_HOST_PTR(s));
    }
    s->mmu_raw = s->mmu_mask = 0;
    imx95_mali_mmu_irq(s);
    s->irq_raw = 0;
    s->irq_mask = 0;
    imx95_mali_update_irq(s);
}

static int imx95_mali_post_load(void *opaque, int version_id)
{
    IMX95MaliState *s = opaque;

    if ((s->irq_raw | s->irq_mask) & ~MALI_IRQ_BITS) {
        return -EINVAL;
    }
    if ((s->mmu_raw | s->mmu_mask) & ~0xffffU) {
        return -EINVAL;
    }
    for (unsigned i = 0; i < MALI_AS_COUNT; i++) {
        imx95_mali_invalidate(&s->as[i]);
    }
    imx95_mali_mmu_irq(s);
    imx95_mali_update_irq(s);
    return 0;
}

static bool imx95_mali_mmu_needed(void *opaque)
{
    return IMX95_MALI(opaque)->mmu_experimental;
}

static const VMStateDescription vmstate_imx95_mali_as = {
    .name = "imx95.mali/as",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(transtab, IMX95MaliAS),
        VMSTATE_UINT64(transcfg, IMX95MaliAS),
        VMSTATE_UINT64(memattr, IMX95MaliAS),
        VMSTATE_UINT64(lockaddr, IMX95MaliAS),
        VMSTATE_UINT64(active_transtab, IMX95MaliAS),
        VMSTATE_UINT64(active_transcfg, IMX95MaliAS),
        VMSTATE_UINT64(active_memattr, IMX95MaliAS),
        VMSTATE_UINT64(faultaddr, IMX95MaliAS),
        VMSTATE_UINT32(faultstatus, IMX95MaliAS),
        VMSTATE_BOOL(locked, IMX95MaliAS),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_imx95_mali_mmu = {
    .name = "imx95.mali/mmu-experimental",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = imx95_mali_mmu_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mmu_raw, IMX95MaliState),
        VMSTATE_UINT32(mmu_mask, IMX95MaliState),
        VMSTATE_STRUCT_ARRAY(as, IMX95MaliState, MALI_AS_COUNT, 0,
                             vmstate_imx95_mali_as, IMX95MaliAS),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_imx95_mali = {
    .name = TYPE_IMX95_MALI,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = imx95_mali_post_load,
    .subsections = (const VMStateDescription * const []) {
        &vmstate_imx95_mali_mmu,
        NULL
    },
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(irq_raw, IMX95MaliState),
        VMSTATE_UINT32(irq_mask, IMX95MaliState),
        VMSTATE_END_OF_LIST()
    },
};

static const MemoryRegionOps imx95_mali_ops = {
    .read = imx95_mali_read,
    .write = imx95_mali_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 8 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void imx95_mali_realize(DeviceState *dev, Error **errp)
{
    IMX95MaliState *s = IMX95_MALI(dev);

    if (s->mcu_experimental && !s->mmu_experimental) {
        error_setg(errp, "mcu-experimental requires mmu-experimental");
        return;
    }
    if (s->mcu_experimental && current_machine->smp.max_cpus < 9) {
        error_setg(errp, "mcu-experimental requires FRDM -smp 8,maxcpus=9");
        return;
    }
    s->gpu_id = MALI_GPU_ID_G310;
    memory_region_init_io(&s->iomem, OBJECT(dev), &imx95_mali_ops, s,
                          TYPE_IMX95_MALI, MALI_MMIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->mmu_irq);
    for (unsigned i = 0; i < MALI_AS_COUNT; i++) {
        IMX95MaliAS *as = &s->as[i];
        g_autofree char *name = g_strdup_printf("mali-as%u", i);

        as->gpu = s;
        as->number = i;
        memory_region_init_iommu(&as->iommu, sizeof(as->iommu),
                                 TYPE_IMX95_MALI_IOMMU, OBJECT(dev), name,
                                 UINT64_MAX);
        address_space_init(&as->dma_as, MEMORY_REGION(&as->iommu), name);
    }
    if (s->mmu_experimental) {
        memory_region_init(&s->mcu_map_memory, OBJECT(s), "mali-mcu-map",
                           0x04000000);
        for (unsigned i = 0; i < MALI_AS_COUNT; i++) {
            g_autofree char *name = g_strdup_printf("mali-mcu-map-as%u", i);

            memory_region_init_alias(&s->mcu_map_alias[i], OBJECT(s), name,
                                     MEMORY_REGION(&s->as[i].iommu), 0,
                                     0x04000000);
            memory_region_set_enabled(&s->mcu_map_alias[i], false);
            memory_region_add_subregion_overlap(&s->mcu_map_memory, 0,
                                                &s->mcu_map_alias[i], i);
        }
    }
    if (s->mcu_experimental) {
        memory_region_init(&s->mcu_memory, OBJECT(s), "mali-mcu-memory", 1ULL << 32);
        memory_region_init_alias(&s->mcu_window, OBJECT(s), "mali-mcu-as0",
                                 MEMORY_REGION(&s->as[0].iommu), 0, 0x08000000);
        memory_region_set_enabled(&s->mcu_window, false);
        memory_region_add_subregion(&s->mcu_memory, 0, &s->mcu_window);
        memory_region_add_subregion(&s->mcu_memory, 0x08000000,
                                    &s->mcu_map_memory);
        memory_region_init_io(&s->mcu_mmio, OBJECT(s), &imx95_mali_mcu_ops, s,
                              "mali-mcu-internal", 0x40000);
        memory_region_add_subregion(&s->mcu_memory, 0x40000000, &s->mcu_mmio);
        s->mcu_clock = clock_new(OBJECT(s), "mcu-clock");
        clock_set_hz(s->mcu_clock, 1000000000);
        object_initialize_child(OBJECT(s), "mcu", &s->mcu, TYPE_ARMV7M);
        qdev_prop_set_string(DEVICE(&s->mcu), "cpu-type", ARM_CPU_TYPE_NAME("cortex-m7"));
        qdev_prop_set_uint32(DEVICE(&s->mcu), "mpu-ns-regions", 16);
        qdev_prop_set_uint32(DEVICE(&s->mcu), "num-irq", 32);
        qdev_prop_set_bit(DEVICE(&s->mcu), "start-powered-off", true);
        qdev_prop_set_bit(DEVICE(&s->mcu), "vfp", false);
        qdev_connect_clock_in(DEVICE(&s->mcu), "cpuclk", s->mcu_clock);
        object_property_set_link(OBJECT(&s->mcu), "memory", OBJECT(&s->mcu_memory),
                                 &error_abort);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->mcu), errp)) {
            return;
        }
        error_setg(&s->mcu_migration_blocker,
                   "Experimental Mali MCU lifecycle/migration is not implemented");
        if (migrate_add_blocker(&s->mcu_migration_blocker, errp)) {
            return;
        }
    }
}

static void imx95_mali_unrealize(DeviceState *dev)
{
    IMX95MaliState *s = IMX95_MALI(dev);

    migrate_del_blocker(&s->mcu_migration_blocker);
    for (unsigned i = 0; i < MALI_AS_COUNT; i++) {
        address_space_destroy(&s->as[i].dma_as);
    }
}

static const Property imx95_mali_properties[] = {
    DEFINE_PROP_BOOL("mmu-experimental", IMX95MaliState, mmu_experimental, false),
    DEFINE_PROP_BOOL("mcu-experimental", IMX95MaliState, mcu_experimental, false),
};

static void imx95_mali_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, imx95_mali_properties);
    dc->realize = imx95_mali_realize;
    dc->unrealize = imx95_mali_unrealize;
    dc->vmsd = &vmstate_imx95_mali;
    device_class_set_legacy_reset(dc, imx95_mali_reset);
    dc->desc = "NXP i.MX 95 Mali GPU (ID and GPU IRQ only)";
}

static const TypeInfo imx95_mali_info = {
    .name           = TYPE_IMX95_MALI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(IMX95MaliState),
    .class_init     = imx95_mali_class_init,
};

static int imx95_mali_attrs_to_index(IOMMUMemoryRegion *iommu, MemTxAttrs attrs)
{
    /* Index 0 is unprivileged, also the conservative unspecified default.
     * TCG calls this method directly when the MCU accesses an IOMMU window.
     */
    return !attrs.unspecified && !attrs.user;
}

static int imx95_mali_num_indexes(IOMMUMemoryRegion *iommu)
{
    return MALI_IOMMU_INDEX_COUNT;
}

static void imx95_mali_iommu_class_init(ObjectClass *klass, const void *data)
{
    IOMMUMemoryRegionClass *imrc = IOMMU_MEMORY_REGION_CLASS(klass);

    imrc->tcg_access_check = true;
    imrc->translate = imx95_mali_translate;
    imrc->attrs_to_index = imx95_mali_attrs_to_index;
    imrc->num_indexes = imx95_mali_num_indexes;
}

static const TypeInfo imx95_mali_iommu_info = {
    .name = TYPE_IMX95_MALI_IOMMU,
    .parent = TYPE_IOMMU_MEMORY_REGION,
    .class_init = imx95_mali_iommu_class_init,
};

static void imx95_mali_register_types(void)
{
    type_register_static(&imx95_mali_iommu_info);
    type_register_static(&imx95_mali_info);
}

type_init(imx95_mali_register_types)
