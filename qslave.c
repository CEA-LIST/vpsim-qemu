/*
 * @file  qslave.c
 *
 * @authors  Amir Charif
 *           Arief Wicaksana
 *			 Mohamed Benazouz
 *
 * @version  1.1
 *
 * @brief  Implementation of the VPSim ModelProvider interface for QEMU.
 *
 * Copyright © 2024 CEA, LIST - All rights reserved.
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.1 or later.
 * See the COPYING.LIB file in the top-level directory.
 *
 * Software is distributed by CEA on an "AS IS" basis, WITHOUT WARRANTY OF
 * ANY KIND either express or implied.
 */

#include "qslave.h"
#include "net/net.h"
#include "sysemu/sysemu.h"
#include "hw/loader.h"
#include "qemu/error-report.h"
#include "hw/intc/arm_gicv3_common.h"
#include "hw/irq.h"
#include "hw/firmware/smbios.h"
#include "kvm_arm.h"
#include "migration/vmstate.h"
#include "hw/char/pl011.h"
#include "hw/boards.h"
#include "hw/arm/boot.h"

#define ARCH_GIC_MAINT_IRQ  9

#define ARCH_TIMER_VIRT_IRQ   11
#define ARCH_TIMER_S_EL1_IRQ  13
#define ARCH_TIMER_NS_EL1_IRQ 14
#define ARCH_TIMER_NS_EL2_IRQ 10

#define VIRTUAL_PMU_IRQ 7

#define PPI(irq) ((irq) + 16)

uint64_t qslave_quantum;

ReadCb _qslave_default_read_cb;
WriteCb _qslave_default_write_cb;
SyncCb _qslave_sync_cb;

void (*_qslave_unlock_cb) (void*);
void (*_qslave_wait_unlock_cb) (void*);

void* _qslave_mp;

MemoryRegionOps _qslave_ops = {
		.read  = modelprovider_read_default,
		.write = modelprovider_write_default
};

MemoryRegion* qslave_sec_mem=NULL;

void* _proxies[MAX_CPUS];
uint64_t _start_pcs[MAX_CPUS];

void* __qslave_gic_handle=NULL;
uint64_t _qslave_gic_dist_base;
uint64_t _qslave_gic_cpu_base;
uint64_t _qslave_gic_hyp_base;
uint64_t _qslave_gic_vcpu_base;

void* __qslave_gicv3_handle=NULL;
uint64_t __qslave_gicv3_dist_base;
uint64_t __qslave_gicv3_cpu_base;
uint64_t __qslave_gicv3_hyp_base;
uint64_t __qslave_gicv3_vcpu_base;

#define MAX_REDIST 16
struct qslave_redist_t {
	uint64_t base;
	uint64_t size;
} __qslave_gicv3_redists [MAX_REDIST];
static uint64_t __qslave_gicv3_n_redist=0;

SysBusDevice * _qslave_deferred_irq_connections[NUM_IRQS];

typedef struct {
	char name[256];
	void* handle;
	void* (*init)(void*,char*,uint64_t,int,ReadCb*,WriteCb*);
} qslave_special_initiator;

void (*__qslave_show_cpu_function)(void* handle)=NULL;

qemu_irq __qslave_in_irqs[NUM_IRQS];

static void create_gicv2(qslave_special_initiator* inits);
static void create_gicv3(qslave_special_initiator* inits);

static void* __qslave_gic_region(int region, void* gicdev, char* name, uint64_t base, ReadCb* rd, WriteCb* wr) {
	if (!__qslave_gic_handle)
		create_gicv2(NULL);
	if (!gicdev)
		gicdev=__qslave_gic_handle;
	SysBusDevice* gicbusdev = SYS_BUS_DEVICE(gicdev);
	sysbus_mmio_map(gicbusdev, region, base);
	MemoryRegion* dev_region = sysbus_mmio_get_region(gicbusdev, region);
	*rd = dev_region->ops->read;
	*wr = dev_region->ops->write;
	return gicdev;
}

static void* qslave_init_gicv2_dist(void* gicdev,char* name,
		uint64_t base,
		int irq,
		ReadCb* rd,
		WriteCb* wr) {
	_qslave_gic_dist_base=base;
	return __qslave_gic_region(0, gicdev, name, base, rd, wr);
}

static void* qslave_init_gicv2_cpu(void* gicdev,char* name,
		uint64_t base,
		int irq,
		ReadCb* rd,
		WriteCb* wr) {
	_qslave_gic_cpu_base=base;
	return __qslave_gic_region(1, gicdev, name, base, rd, wr);
}

static void* qslave_init_gicv2_hyp(void* gicdev,char* name,
		uint64_t base,
		int irq,
		ReadCb* rd,
		WriteCb* wr) {
	_qslave_gic_hyp_base=base;
	return __qslave_gic_region(2, gicdev, name, base, rd, wr);
}

static void* qslave_init_gicv2_vcpu(void* gicdev,char* name,
		uint64_t base,
		int irq,
		ReadCb* rd,
		WriteCb* wr) {
	_qslave_gic_vcpu_base=base;
	return __qslave_gic_region(3, gicdev, name, base, rd, wr);
}

static void* qslave_init_gicv3_redist(void* gicdev,char* name,
		uint64_t base,
		int irq,
		ReadCb* rd,
		WriteCb* wr) {
	if(!__qslave_gicv3_handle) {
		create_gicv3(NULL);
	}
	if (__qslave_gicv3_n_redist > MAX_REDIST) {
		fprintf(stderr, "Too many redistributors for gicv3 !\n");
		exit(1);
	}
	struct qslave_redist_t red;
	red.base=base;
	red.size=GICV3_REDIST_SIZE*irq;
	__qslave_gicv3_redists[__qslave_gicv3_n_redist++] = red;
	return __qslave_gicv3_handle;
}

static void* qslave_init_gicv3_dist(void* gicdev,char* name,
		uint64_t base,
		int irq,
		ReadCb* rd,
		WriteCb* wr) {
	if(!__qslave_gicv3_handle) {
		create_gicv3(NULL);
	}
	__qslave_gicv3_dist_base=base;
	return __qslave_gicv3_handle;
}

static void* qslave_init_gicv3_cpu(void* gicdev,char* name,
		uint64_t base,
		int irq,
		ReadCb* rd,
		WriteCb* wr) {
	if(!__qslave_gicv3_handle) {
		create_gicv3(NULL);
	}
	__qslave_gicv3_cpu_base=base;
	return __qslave_gicv3_handle;
}

static void* qslave_init_gicv3_hyp(void* gicdev,char* name,
		uint64_t base,
		int irq,
		ReadCb* rd,
		WriteCb* wr) {
	if(!__qslave_gicv3_handle) {
		create_gicv3(NULL);
	}
	__qslave_gicv3_hyp_base=base;
	return __qslave_gicv3_handle;
}

static void* qslave_init_gicv3_vcpu(void* gicdev,char* name,
		uint64_t base,
		int irq,
		ReadCb* rd,
		WriteCb* wr) {
	if(!__qslave_gicv3_handle) {
		create_gicv3(NULL);
	}
	__qslave_gicv3_vcpu_base=base;
	return __qslave_gicv3_handle;
}

static void* qslave_init_pl011(void* gicdev,char* name,
		uint64_t base,
		int irq,
		ReadCb* rd,
		WriteCb* wr) {
	DeviceState *dev = qdev_new(TYPE_PL011);
	SysBusDevice *s = SYS_BUS_DEVICE(dev);

	qdev_prop_set_chr(dev, "chardev", serial_hd(0));
	sysbus_realize_and_unref(s, &error_fatal);
	memory_region_add_subregion(get_system_memory(), base,
								sysbus_mmio_get_region(s, 0));
	_qslave_deferred_irq_connections[irq]=s;

	return dev;
}

static void* qslave_init_pl031(void* gicdev,char* name,
		uint64_t base,
		int irq,
		ReadCb* rd,
		WriteCb* wr) {
	DeviceState *dev = qdev_new("pl031");
	SysBusDevice *s = SYS_BUS_DEVICE(dev);

	sysbus_realize_and_unref(s, &error_fatal);
	memory_region_add_subregion(get_system_memory(), base,
								sysbus_mmio_get_region(s, 0));
	_qslave_deferred_irq_connections[irq]=s;

	return dev;
}

static void* qslave_init_flash(void* hdl,char* name,
		uint64_t base,
		int irq,
		ReadCb* rd,
		WriteCb* wr) {
	DeviceState *dev = qdev_new("cfi.pflash01");
	const uint64_t sectorlength = 256 * 1024;

	uint64_t size=(uint64_t)irq / sectorlength;
	printf("Size is %ld\n", size);
	qdev_prop_set_uint32(dev, "num-blocks", size);
	qdev_prop_set_uint64(dev, "sector-length", sectorlength);
	qdev_prop_set_uint8(dev, "width", 4);
	qdev_prop_set_uint8(dev, "device-width", 2);
	qdev_prop_set_bit(dev, "big-endian", false);
	qdev_prop_set_uint16(dev, "id0", 0x89);
	qdev_prop_set_uint16(dev, "id1", 0x18);
	qdev_prop_set_uint16(dev, "id2", 0x00);
	qdev_prop_set_uint16(dev, "id3", 0x00);
	qdev_prop_set_string(dev, "name", name);
	sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);


	memory_region_add_subregion(get_system_memory(), base,
								sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0));


    return dev;
}



static void qslave_build_smbios(FWCfgState *fw_cfg)
{
    uint8_t *smbios_tables, *smbios_anchor;
    size_t smbios_tables_len, smbios_anchor_len;
    const char *product = "QSLAVE Custom Virtual Machine";

    smbios_set_defaults("QEMU", product,
                        "1.0", false,
                        true, SMBIOS_ENTRY_POINT_TYPE_64);

    smbios_get_tables(current_machine, NULL, 0, &smbios_tables, &smbios_tables_len,
                      &smbios_anchor, &smbios_anchor_len, &error_fatal);

    if (smbios_anchor) {
        fw_cfg_add_file(fw_cfg, "etc/smbios/smbios-tables",
                        smbios_tables, smbios_tables_len);
        fw_cfg_add_file(fw_cfg, "etc/smbios/smbios-anchor",
                        smbios_anchor, smbios_anchor_len);
    }
}

// from virt
static FWCfgState *create_fw_cfg(uint64_t base,uint64_t size)
{
    FWCfgState *fw_cfg;

    fw_cfg = fw_cfg_init_mem_wide(base + 8, base, 8, base + 16, &address_space_memory);
    fw_cfg_add_i16(fw_cfg, FW_CFG_NB_CPUS, (uint16_t)current_machine->smp.cpus);

    rom_set_fw(fw_cfg);

    qslave_build_smbios(fw_cfg);

    return fw_cfg;
}

static void* qslave_init_fw_cfg(void* Wdev,char* name,
		uint64_t base,
		int irq,
		ReadCb* rd,
		WriteCb* wr) {
	return create_fw_cfg(base,0x18);
}

// from virt
static void* create_pcie(void* devh, char* name,
		uint64_t base_addr,
		int irqn,
		ReadCb* rd,
		WriteCb* wr)
{
    hwaddr base_mmio = base_addr;
    hwaddr size_mmio = 0x2eff0000;
    hwaddr base_pio = base_mmio + size_mmio;
    hwaddr size_pio = 0x00010000;
    hwaddr base_ecam = base_pio + size_pio;
    hwaddr size_ecam = 0x01000000;
    int irq = irqn;
    MemoryRegion *mmio_alias;
    MemoryRegion *mmio_reg;
    MemoryRegion *ecam_alias;
    MemoryRegion *ecam_reg;
    DeviceState *dev;
    int i;
    PCIHostState *pci;

    dev = qdev_new(TYPE_GPEX_HOST);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    ecam_alias = g_new0(MemoryRegion, 1);
    ecam_reg = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_init_alias(ecam_alias, OBJECT(dev), "pcie-ecam",
                             ecam_reg, 0, size_ecam);
    memory_region_add_subregion(get_system_memory(), base_ecam, ecam_alias);

    mmio_alias = g_new0(MemoryRegion, 1);
    mmio_reg = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 1);
    memory_region_init_alias(mmio_alias, OBJECT(dev), "pcie-mmio",
                             mmio_reg, base_mmio, size_mmio);
    memory_region_add_subregion(get_system_memory(), base_mmio, mmio_alias);

	MemoryRegion *high_mmio_alias = g_new0(MemoryRegion, 1);

	memory_region_init_alias(high_mmio_alias, OBJECT(dev), "pcie-mmio-high",
							 mmio_reg, 0x8000000000ULL, 0x8000000000ULL);
	memory_region_add_subregion(get_system_memory(), 0x8000000000ULL,
								high_mmio_alias);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 2, base_pio);

    for (i = 0; i < GPEX_NUM_IRQS; i++) {
        _qslave_deferred_irq_connections[irq+i] = SYS_BUS_DEVICE(dev);
        gpex_set_irq_num(GPEX_HOST(dev), i, irq + i);
    }

    pci = PCI_HOST_BRIDGE(dev);
	pci->bypass_iommu = false;
    if (pci->bus) {
        for (i = 0; i < nb_nics; i++) {
            NICInfo *nd = &nd_table[i];

            if (!nd->model) {
                nd->model = g_strdup("virtio");
            }

            pci_nic_init_nofail(nd, pci->bus, nd->model, NULL);
        }
    }

    return dev;
}


qslave_special_initiator _qslave_special_initiators[] = {
		{.name="gicv2_dist", .handle=NULL, .init=qslave_init_gicv2_dist},
		{.name="gicv2_cpu", .handle=NULL, .init=qslave_init_gicv2_cpu},
		{.name="gicv2_hyp", .handle=NULL, .init=qslave_init_gicv2_hyp},
		{.name="gicv2_vcpu", .handle=NULL, .init=qslave_init_gicv2_vcpu},
		{.name="gicv3_dist", .handle=NULL, .init=qslave_init_gicv3_dist},
		{.name="gicv3_redist", .handle=NULL, .init=qslave_init_gicv3_redist},
		{.name="gicv3_cpu", .handle=NULL, .init=qslave_init_gicv3_cpu},
		{.name="gicv3_hyp", .handle=NULL, .init=qslave_init_gicv3_hyp},
		{.name="gicv3_vcpu", .handle=NULL, .init=qslave_init_gicv3_vcpu},
		{.name="pl011", .handle=NULL, .init=qslave_init_pl011},
		{.name="fw_cfg", .handle=NULL, .init=qslave_init_fw_cfg},
		{.name="pl031", .handle=NULL, .init=qslave_init_pl031},
		{.name="cfi_flash_0", .handle=NULL, .init=qslave_init_flash},
		{.name="cfi_flash_1", .handle=NULL, .init=qslave_init_flash},
		{.name="pcie", .handle=NULL, .init=create_pcie}
};

static void create_gicv2(qslave_special_initiator* inits)
{
    DeviceState *gicdev;
    const char *gictype;
    int type = 2;
    int i;

    gictype = "arm_gic";

    if(__qslave_gicv3_handle)
    {
        fprintf(stderr, "Conflict: Creating a GICv2 while a GICv3 exists for this cluster !\n");
        exit(1);
    }

    gicdev = qdev_new(gictype);
    qdev_prop_set_uint32(gicdev, "revision", type);
    qdev_prop_set_uint32(gicdev, "num-cpu", current_machine->smp.cpus);

    qdev_prop_set_uint32(gicdev, "num-irq", NUM_IRQS + 32);
    qdev_prop_set_bit(gicdev, "has-security-extensions", true);


    qdev_prop_set_bit(gicdev, "has-virtualization-extensions", true);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(gicdev), &error_fatal);
    __qslave_gic_handle = gicdev;

    for (i = 0; i < NUM_IRQS; i++) {
		__qslave_in_irqs[i] = qdev_get_gpio_in(gicdev, i);
	}
}

static void create_gicv3(qslave_special_initiator* inits)
{
    DeviceState *gicdev;
    const char *gictype;
    int type = 3;

    if(__qslave_gic_handle)
	{
		fprintf(stderr, "Conflict: Creating a GICv3 while a GICv2 exists for this cluster !\n");
		exit(1);
	}

    gictype = "arm-gicv3";

    gicdev = qdev_new(gictype);
    qdev_prop_set_uint32(gicdev, "revision", type);
    qdev_prop_set_uint32(gicdev, "num-cpu", current_machine->smp.cpus);

    qdev_prop_set_uint32(gicdev, "num-irq", NUM_IRQS + 32);
    qdev_prop_set_bit(gicdev, "has-security-extensions", false);


	__qslave_gicv3_handle = gicdev;
}

static qslave_special_initiator* qslave_get_special_initiator(const char* name) {
	int i=0;

	for (i = 0; i < ARRAY_SIZE(_qslave_special_initiators); i++) {
		if (!strcmp(name, _qslave_special_initiators[i].name))
			return &_qslave_special_initiators[i];
	}
	return NULL;
}

void qslave_cpu_print_stats(int index);

// FIXME: remove the printfs and use the proper stats API.
static void qslave_aarch64_show(void* cpu) {
	ARMCPU * armcpu = ARM_CPU(cpu);
	CPUARMState *env = ((CPUState*)cpu)->env_ptr;
    uint32_t cur_el = arm_current_el(env);
	int index;

	printf("======== CPU index : %d ========\n", ((CPUState*)cpu)->cpu_index); // Checking index of cpu
	printf("### Current Exception Level = EL%d\n", cur_el);
	printf("### Registers: \n");

	for (index = 0; index < 31; index++)
	{
		if (index % 3 == 0) {
			printf("\n");
		}
		printf("x%02d = %016lx\t", index, armcpu->env.xregs[index]);
	}


	printf("\npc = %016lx\n", armcpu->env.pc);

	printf("\npstate = %x\n", armcpu->env.pstate);
	printf("aarch64 = %x\n",  armcpu->env.aarch64);
	printf("uncached_cpsr = %x\n", armcpu->env.uncached_cpsr);
	printf("spsr = %x\n", armcpu->env.spsr);

	for (index = 0; index < 8; index++) {
		if (index % 2 == 0)
			printf("\n");
		printf("banked_spsr[%d] : %016lx\t", index, armcpu->env.banked_spsr[index]);
	}
	printf("\n");
	for (index = 0; index < 8; index++) {
		if (index % 2 == 0)
			printf("\n");
		printf("banked_r13[%d] : %08x\t", index, armcpu->env.banked_r13[index]);
	}
	printf("\n");
	for (index = 0; index < 8; index++) {
		if (index % 2 == 0)
			printf("\n");
		printf("banked_r14[%d] : %08x\t", index, armcpu->env.banked_r14[index]);
	}
	printf("\n");
	for (index = 0; index < 5; index++) {
		if (index % 2 == 0)
			printf("\n");
		printf("usr_regs[%d] : %08x\t", index, armcpu->env.usr_regs[index]);
	}
	printf("\n");
	for (index = 0; index < 5; index++) {
		if (index % 2 == 0)
			printf("\n");
		printf("fiq_regs[%d] : %08x\t", index, armcpu->env.fiq_regs[index]);
	}

	printf("\n");
	for (index = 0; index < 4; index++) {
		if (index % 2 == 0)
			printf("\n");
		printf("elr_el[%d] : %016lx\t", index, armcpu->env.elr_el[index]);
	}

	printf("\n");
	for (index = 0; index < 4; index++) {
		if (index % 2 == 0)
			printf("\n");
		printf("sp_el[%d] : %016lx\t", index, armcpu->env.sp_el[index]);
	}

	printf("\n\n\n### Current statistics summary:\n");
	qslave_cpu_print_stats(((CPUState*)cpu)->cpu_index);
}

void modelprovider_show_cpu(void* handle) {
	if (__qslave_show_cpu_function==NULL)
	{
		fprintf(stderr, "QSlave cannot show this cpu.\n");
		return;
	}
	__qslave_show_cpu_function(handle);
}


void modelprovider_interrupt(int index, int value) {
	qemu_set_irq(__qslave_in_irqs[index], value);
}


void modelprovider_set_default_read_callback(ReadCb cb) {
	_qslave_default_read_cb = cb;
}

void modelprovider_set_default_write_callback(WriteCb cb) {
	_qslave_default_write_cb = cb;
}

void modelprovider_set_sync_callback(SyncCb cb) {
	_qslave_sync_cb = cb;
}

__thread bool qslave_run_start = false;

void qslave_yield(void* opaque, uint64_t executed, int wfi) {
	CPUState* cpu = (CPUState*)opaque;
	_qslave_sync_cb(_proxies[cpu->cpu_index], executed, wfi);
}

uint64_t modelprovider_read_default(void *opaque,
                     hwaddr addr,
                     unsigned size) {
	MemoryRegion* mr = opaque;
	return _qslave_default_read_cb(_proxies[current_cpu->cpu_index], addr+mr->addr, size);
}

void modelprovider_write_default(void *opaque,
                  hwaddr addr,
                  uint64_t data,
                  unsigned size) {
	MemoryRegion* mr = opaque;
	_qslave_default_write_cb(_proxies[current_cpu->cpu_index], addr+mr->addr, data, size);
}

uint64_t modelprovider_get_start_pc(int index) { return _start_pcs[index]; }

static void _qslave_ioaccess_notify_functor(uint32_t device, int write, void* phys, uint64_t virt, uint64_t size, uint64_t tag) {
    //Modify the following execution counter (qemu timestamp) if it is not correct from within the context this function is called!
    uint64_t executed = current_cpu->icount_budget - (cpu_neg(current_cpu)->icount_decr.u16.low + current_cpu->icount_extra);
    qslave_ioaccess_notify_model(device, executed, write, phys, virt, size, tag);
}

void modelprovider_register_ioaccess_callback(IOAccessCb cb) {
	qslave_ioaccess_notify_model=cb;
	qslave_ioaccess_notify=_qslave_ioaccess_notify_functor;
}

IOAccessCbInternal qslave_ioaccess_notify=NULL;
IOAccessCb qslave_ioaccess_notify_model=NULL;

static void _qslave_mem_notify_functor(int write, void* phys, uint64_t virt, uint64_t size) {
        uint64_t executed = current_cpu->icount_budget - (cpu_neg(current_cpu)->icount_decr.u16.low + current_cpu->icount_extra);
	qslave_mem_notify_model(_proxies[current_cpu->cpu_index], executed,
			write, phys, virt, size);
}

void modelprovider_register_main_mem_callback(MainMemCb cb) {
	qslave_mem_notify_model=cb;
	qslave_mem_notify=_qslave_mem_notify_functor;
}

void modelprovider_unregister_main_mem_callback(void) {
	qslave_mem_notify_model=NULL;
	qslave_mem_notify=NULL;
}

ICacheMissCb qslave_icache_miss_cb=NULL;

static uint64_t _icache_misses[MAX_CPUS];
uint64_t qslave_get_icache_misses(int index) {
	return _icache_misses[index];
}

void modelprovider_register_icache_miss_cb(ICacheMissCb cb) {
	qslave_icache_miss_cb=cb;
}

void qslave_icache_miss(uint64_t addr, unsigned size, int* u) {
	int** v=(int**) u;
	if (! *v || ! **v) {
		_icache_misses[current_cpu->cpu_index] = qslave_icache_miss_cb(_proxies[current_cpu->cpu_index],addr,size,u);
	}
	assert(*v != NULL);
}

MainMemCbInternal qslave_mem_notify=NULL;
MainMemCb qslave_mem_notify_model=NULL;

FillBiasCb _modelprovider_fill_bias_cb=NULL;
void modelprovider_register_fill_bias_cb(FillBiasCb cb) {
	_modelprovider_fill_bias_cb=cb;
}
void qslave_fill_biases(uint64_t* ts) {
	_modelprovider_fill_bias_cb(ts,current_machine->smp.cpus);
}


void* modelprovider_create_internal_dev_default(char* name,
										uint64_t base,
										int irq,
										ReadCb* rd,
										WriteCb* wr) {
	qslave_special_initiator* si = qslave_get_special_initiator(name);

	if (si) {
		return si->init(si->handle,name,base,irq,rd,wr);
	} else {
		DeviceState *dev = qdev_new(name);
		SysBusDevice *s = SYS_BUS_DEVICE(dev);

		sysbus_realize_and_unref(s, &error_fatal);

		MemoryRegion* dev_region = sysbus_mmio_get_region(s, 0);

		*rd = dev_region->ops->read;
		*wr = dev_region->ops->write;

		memory_region_add_subregion(get_system_memory(), base, dev_region);

		if (_qslave_deferred_irq_connections[irq])
		{
			fprintf(stderr, "%s: IRQ line %d already taken.\n", name, irq);
			exit(1);
		}
		_qslave_deferred_irq_connections[irq]=s;

		if (!rd || !wr) {
			fprintf(stderr, "QSlave: Problem with internal device subregion ops.\n");
			exit(-1);
		}
		return dev;
	}
}

void modelprovider_declare_external_dev(char* name, uint64_t base, uint64_t size) {
	MemoryRegion* dev_region = g_new(MemoryRegion, 1);
	memory_region_init_io(dev_region, NULL, &_qslave_ops, NULL, name, size);
	if (strcmp(name, "Monitor0"))
		memory_region_add_subregion_overlap(get_system_memory(), base, dev_region, -1);
	else
		memory_region_add_subregion_overlap(get_system_memory(), base, dev_region, 1);
	dev_region->opaque=dev_region;
}

void modelprovider_declare_external_ram(char* name, uint64_t base, uint64_t size, void* data) {
	MemoryRegion* ram_region = g_new(MemoryRegion, 1);
	memory_region_init_ram_ptr ( ram_region, NULL, name, size, data );
	vmstate_register_ram_global ( ram_region );
	memory_region_add_subregion(get_system_memory(), base, ram_region);
}

void modelprovider_register_unlock(void(*cb)(void*),void* mp) {
	_qslave_unlock_cb = cb;
	_qslave_mp=mp;
}

void modelprovider_register_wait_unlock(void(*cb)(void*),void* mp) {
	_qslave_wait_unlock_cb = cb;
	_qslave_mp=mp;
}

void modelprovider_unlock(void) {
	CPUState* temp = current_cpu;
	_qslave_unlock_cb(_qslave_mp);
	current_cpu = temp;
}

void modelprovider_wait_unlock(void) {
	CPUState* temp = current_cpu;
	_qslave_wait_unlock_cb(_qslave_mp);
	current_cpu = temp;
}

void* modelprovider_create_internal_cpu(void *proxy, char* type, int index, uint64_t start_pc, int secure, int start_off) {
	Object *cpuobj;
	CPUState *cs;

	cpuobj = object_new(type);
	cs = CPU(cpuobj);
	cs->cpu_index = index;

    _proxies[index] = proxy;
    _start_pcs[index]=start_pc;

    modelprovider_cpu_register_stats(index);

	object_property_set_link(cpuobj, "memory", OBJECT(get_system_memory()),
	                                 &error_abort);
    object_property_set_bool(cpuobj, "has_el3", secure, NULL);
    object_property_set_bool(cpuobj, "has_el2", secure, NULL);
    object_property_set_int(cpuobj, "rvbar", start_pc, NULL);
    object_property_set_int(cpuobj, "mp-affinity", arm_cpu_mp_affinity(index, 16), NULL);
    object_property_set_bool(cpuobj, "pmu", true, NULL);

    object_property_set_bool(cpuobj, "start-powered-off", start_off, NULL);

    __qslave_show_cpu_function = qslave_aarch64_show;
    _icache_misses[index]=0;

    object_property_set_int(cpuobj, "psci-conduit", (secure?
        QEMU_PSCI_CONDUIT_DISABLED : QEMU_PSCI_CONDUIT_SMC), NULL);

    object_property_set_bool(cpuobj, "realized", true, NULL);
    object_unref(cpuobj);

    return cs;
}

IOAccessStatGetter __qslave_get_ioaccess_stat_cb=NULL;

static inline uint64_t GET_IOAccess_STAT(uint32_t device, enum IOAccessStat type) {
	return __qslave_get_ioaccess_stat_cb(device, type);
}

void modelprovider_register_ioaccess_stat_cb(IOAccessStatGetter cb) {
	__qslave_get_ioaccess_stat_cb=cb;
}

uint64_t qslave_get_ioaccess_read(uint32_t device) {
	if(!__qslave_get_ioaccess_stat_cb) return 0;
	return GET_IOAccess_STAT(device, IOACCESS_READ);
}

uint64_t qslave_get_ioaccess_write(uint32_t device) {
	if(!__qslave_get_ioaccess_stat_cb) return 0;
	return GET_IOAccess_STAT(device, IOACCESS_WRITE);
}

IOAccessGetDelayCb _modelprovider_ioaccess_get_delay_cb=NULL;
void modelprovider_register_ioaccess_get_delay_cb(IOAccessGetDelayCb cb) {
	_modelprovider_ioaccess_get_delay_cb=cb;
}

/**
* Retrieve timing information about the accomplishment of an IO transfer
*
* @param    device			the device id
* @param    time_stamp		Global VPSim time (in picoseconds) at which the read/write operation was received
* @param    delay			delay to accomplish the operation (in picoseconds), it includes NoC transfer (routing+contention) and memory latency.
* @return	1 if the call succeeded, 0 for unsuccessful call
* @note     No allocation is done inside these functions for time_stamp and delay variables.
*			They are passed as pointers in order to update their values if the call is successful.
*/
char qslave_ioaccess_get_delay(uint32_t device, uint64_t* time_stamp, uint64_t* delay, uint64_t* tag) {
	return _modelprovider_ioaccess_get_delay_cb(device,time_stamp,delay,tag);
}
/**
* Busy waiting version of the get_delay function
*/
void qslave_ioaccess_busy_get_delay(uint32_t device, uint64_t* time_stamp, uint64_t* delay, uint64_t* tag) {
	while(!qslave_ioaccess_get_delay(device, time_stamp, delay, tag));
}

OuterStatGetter __qslave_get_outer_stat_cb=NULL;
static inline uint64_t GET_OUTER_STAT(int idx,enum OuterStat type) {if(idx>=0) return __qslave_get_outer_stat_cb(idx,type);
else{ int i,k=0;for(i=0;i<current_machine->smp.cpus;i++)k+=__qslave_get_outer_stat_cb(i,type); return k; }}

void modelprovider_register_outer_stat_cb(OuterStatGetter cb) {
	__qslave_get_outer_stat_cb=cb;
}

uint64_t qslave_get_inst_count(int index) {
	return qslave_stat_cpu[index].executed_instructions.v;
}

uint64_t qslave_get_l1_miss(int index) {
	if(!__qslave_get_outer_stat_cb) return 0;
	return GET_OUTER_STAT(index, L1_MISS);
}

uint64_t qslave_get_l2_miss(int index) {
	if(!__qslave_get_outer_stat_cb) return 0;
	return GET_OUTER_STAT(index, L2_MISS);
}

uint64_t qslave_get_l1_wb(int index) {
	if(!__qslave_get_outer_stat_cb) return 0;
	return GET_OUTER_STAT(index, L1_WB);
}

uint64_t qslave_get_l2_wb(int index) {
	if(!__qslave_get_outer_stat_cb) return 0;
	return GET_OUTER_STAT(index, L2_WB);
}

uint64_t qslave_get_l1_ld(int index) {
	if(!__qslave_get_outer_stat_cb) return 0;
	return GET_OUTER_STAT(index, L1_LD);
}

uint64_t qslave_get_l1_st(int index) {
	if(!__qslave_get_outer_stat_cb) return 0;
	return GET_OUTER_STAT(index, L1_ST);
}

uint64_t qslave_get_l2_ld(int index) {
	if(!__qslave_get_outer_stat_cb) return 0;
	return GET_OUTER_STAT(index, L2_LD);
}

uint64_t qslave_get_l2_st(int index) {
	if(!__qslave_get_outer_stat_cb) return 0;
	return GET_OUTER_STAT(index, L2_ST);
}

uint64_t qslave_get_ld(int index) {
	if(index >= 0)
		return qslave_stat_cpu[index].loads.v;
	else {
		int s=0;
		int i;
		for (i = 0; i < current_machine->smp.cpus; i++)
			s+=qslave_stat_cpu[i].loads.v;
		return s;
	}
}

uint64_t qslave_get_st(int index) {
	if(index >= 0)
		return qslave_stat_cpu[index].stores.v;
	else {
		int s=0;
		int i;
		for (i = 0; i < current_machine->smp.cpus; i++)
			s+=qslave_stat_cpu[i].stores.v;
		return s;
	}
}

qslave_cpu_stat qslave_stat_cpu[MAX_CPUS];

void modelprovider_cpu_register_stats(int index) {
    QSLAVE_STAT(index, executed_instructions);
    QSLAVE_STAT(index, executed_fp_instructions);
    QSLAVE_STAT(index, executed_sve_instructions);
    QSLAVE_STAT(index, stores);
    QSLAVE_STAT(index, loads);
    QSLAVE_STAT(index, load_store);
    QSLAVE_STAT(index, count_tlb_hit);
    QSLAVE_STAT(index, count_tlb_miss);
}

qslave_stat_entry* qslave_cpu_get_stat(int index, char* name) {
	qslave_stat_entry* ptr=(qslave_stat_entry*)&qslave_stat_cpu[index];
	int sz=sizeof(qslave_cpu_stat);
	while (sz) {
		if(!strcmp(ptr->name, name))
			return ptr;
		ptr++;
		sz -= sizeof(qslave_stat_entry);
	}
	return NULL;
}

void modelprovider_cpu_get_stats(int index, uint32_t* count, void** entries) {
	*entries=&qslave_stat_cpu[index];
	*count = sizeof(qslave_cpu_stat)/sizeof(qslave_stat_entry);
}

void qslave_cpu_print_stats(int index) {
	qslave_stat_entry* ptr=(qslave_stat_entry*)&qslave_stat_cpu[index];
	int sz=sizeof(qslave_cpu_stat);
	while (sz) {
		printf("%s = %ld\n", ptr->name, ptr->v);
		ptr++;
		sz -= sizeof(qslave_stat_entry);
	}
}

AddVictimCb qslave_add_victim_cb;
void modelprovider_register_add_victim_cb(AddVictimCb cb) {
	qslave_add_victim_cb = cb;
}

Notifier __qslave_notify;
struct arm_boot_info __qslave_bootinfo;

static void __qslave_done(Notifier *notifier, void *data)
{
	return;
}

int __qslave_thread;



void modelprovider_post_init(MachineState *machine) {

	if (machine->kernel_filename) {
		__qslave_bootinfo.ram_size = machine->ram_size;
		__qslave_bootinfo.kernel_filename = machine->kernel_filename;
		__qslave_bootinfo.kernel_cmdline = machine->kernel_cmdline;
		__qslave_bootinfo.initrd_filename = machine->initrd_filename;
		__qslave_bootinfo.board_id = -1;
		__qslave_bootinfo.loader_start = _start_pcs[first_cpu->cpu_index];
		__qslave_bootinfo.skip_dtb_autoload = false;
		__qslave_bootinfo.firmware_loaded = false;
		arm_load_kernel(ARM_CPU(first_cpu), machine, &__qslave_bootinfo);
	}

	if (__qslave_gicv3_handle || __qslave_gic_handle) {
		int gic_version = (__qslave_gicv3_handle? 3: 2);


		DeviceState* gicdev=(__qslave_gicv3_handle?
				__qslave_gicv3_handle: __qslave_gic_handle);
		SysBusDevice* gicbusdev = SYS_BUS_DEVICE(gicdev);
		int i;

		// from virt
		if (gic_version == 3) {
			int n_redist = __qslave_gicv3_n_redist;


			qdev_prop_set_uint32(gicdev, "len-redist-region-count",
					n_redist);

			int cpu_left=current_machine->smp.cpus;
			int reg;
			for (reg = 0; reg < n_redist; reg++) {
				unsigned redist0_capacity =
						__qslave_gicv3_redists[reg].size / GICV3_REDIST_SIZE;
				unsigned redist0_count = MIN(current_machine->smp.cpus, redist0_capacity);
				char region_param[512];
				sprintf(region_param, "redist-region-count[%d]", reg);
				qdev_prop_set_uint32(gicdev, region_param, redist0_count);
				if (cpu_left > 0)
					cpu_left -= redist0_count;
			}

			sysbus_realize_and_unref(gicbusdev, &error_fatal);


			sysbus_mmio_map(gicbusdev, 0, __qslave_gicv3_dist_base);
			for (i = 0; i < n_redist; i++) {
				SysBusDevice* gicbusdev = SYS_BUS_DEVICE(gicdev);
				sysbus_mmio_map(gicbusdev, i+1, __qslave_gicv3_redists[i].base);
			}
		}

		for (i = 0; i < current_machine->smp.cpus; i++) {
			DeviceState *cpudev = DEVICE(qemu_get_cpu(i));
			int ppibase = NUM_IRQS + i * GIC_INTERNAL + GIC_NR_SGIS;
			int irq;

			const int timer_irq[] = {
				[GTIMER_PHYS] = ARCH_TIMER_NS_EL1_IRQ,
				[GTIMER_VIRT] = ARCH_TIMER_VIRT_IRQ,
				[GTIMER_HYP]  = ARCH_TIMER_NS_EL2_IRQ,
				[GTIMER_SEC]  = ARCH_TIMER_S_EL1_IRQ,
			};

			for (irq = 0; irq < ARRAY_SIZE(timer_irq); irq++) {
				qdev_connect_gpio_out(cpudev, irq,
									  qdev_get_gpio_in(gicdev,
													   ppibase + timer_irq[irq]));
			}

			if (gic_version == 2) {
				qemu_irq irqc = qdev_get_gpio_in(gicdev,
												ppibase + ARCH_GIC_MAINT_IRQ);
				sysbus_connect_irq(gicbusdev, i + 4 * current_machine->smp.cpus, irqc);
			} else {
				qemu_irq irq = qdev_get_gpio_in(gicdev,
												ppibase + ARCH_GIC_MAINT_IRQ);
				qdev_connect_gpio_out_named(cpudev, "gicv3-maintenance-interrupt",
											0, irq);

			}

			qdev_connect_gpio_out_named(cpudev, "pmu-interrupt", 0,
										qdev_get_gpio_in(gicdev, ppibase
														 + VIRTUAL_PMU_IRQ));

			sysbus_connect_irq(gicbusdev, i, qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
			sysbus_connect_irq(gicbusdev, i + current_machine->smp.cpus,
							   qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
			sysbus_connect_irq(gicbusdev, i + 2 * current_machine->smp.cpus,
							   qdev_get_gpio_in(cpudev, ARM_CPU_VIRQ));
			sysbus_connect_irq(gicbusdev, i + 3 * current_machine->smp.cpus,
							   qdev_get_gpio_in(cpudev, ARM_CPU_VFIQ));
		}

		int cur_irq=0;
		for (i = 0; i < NUM_IRQS; i++) {
			__qslave_in_irqs[i] = qdev_get_gpio_in(gicdev, i);

			if (_qslave_deferred_irq_connections[i]) {
				if (i > 0 && _qslave_deferred_irq_connections[i] == _qslave_deferred_irq_connections[i-1])
					cur_irq++;
				else cur_irq=0;
				sysbus_connect_irq(_qslave_deferred_irq_connections[i],
						cur_irq, __qslave_in_irqs[i]);
			}
		}

		if (gic_version == 3) {
			const char *itsclass = its_class_name();
			DeviceState *dev;

			if (!strcmp(itsclass, "arm-gicv3-its")) {
				return;
			}

			dev = qdev_new(itsclass);

			object_property_set_link(OBJECT(dev), "parent-gicv3", OBJECT(gicdev),&error_abort);
			sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
			sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, __qslave_gicv3_dist_base+0x500000);
		}
	}

	__qslave_notify.notify = __qslave_done;
	qemu_add_machine_init_done_notifier(&__qslave_notify);
}



static void qslave_machine_init(MachineState *machine)
{
	__qslave_thread = qemu_get_thread_id();
	int i;
	for (i = 0; i < NUM_IRQS; i++) {
		_qslave_deferred_irq_connections[i] = NULL;
	}
}

int modelprovider_thread_id(void) {
	return __qslave_thread;
}

static void qslave_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "qslave";
    mc->init = qslave_machine_init;
    mc->max_cpus = MAX_CPUS;
    mc->minimum_page_bits = 12;
}

static const TypeInfo qslave_machine_type = {
    .name = MACHINE_TYPE_NAME("qslave"),
    .parent = TYPE_MACHINE,
    .class_init = qslave_machine_class_init,
};

static void qslave_machine_register_type(void)
{
    type_register_static(&qslave_machine_type);
}

type_init(qslave_machine_register_type);
