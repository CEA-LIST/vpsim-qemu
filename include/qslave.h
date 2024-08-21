/*
 * @file  qslave.h
 *
 * @authors  Amir Charif
 *           Arief Wicaksana
 *
 * @version  1.0
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

#ifndef _QSLAVE_H_
#define _QSLAVE_H_

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/vfio/vfio-calxeda-xgmac.h"
#include "hw/vfio/vfio-amd-xgbe.h"
#include "hw/display/ramfb.h"
#include "hw/devices.h"
#include "net/net.h"
#include "sysemu/device_tree.h"
#include "sysemu/numa.h"
#include "sysemu/sysemu.h"
#include "sysemu/kvm.h"
#include "hw/compat.h"
#include "hw/loader.h"
#include "exec/address-spaces.h"
#include "qemu/bitops.h"
#include "qemu/error-report.h"
#include "hw/pci-host/gpex.h"
#include "hw/arm/sysbus-fdt.h"
#include "hw/platform-bus.h"
#include "qapi/visitor.h"
#include "standard-headers/linux/input.h"

#define MAX_CPUS 128
#define NUM_IRQS 256

#define QSLAVE_IDLE_TIME_NS 0xffff

typedef uint64_t (*ReadCb)(void *opaque,
                     hwaddr addr,
                     unsigned size);

typedef void (*WriteCb)(void *opaque,
                  hwaddr addr,
                  uint64_t data,
                  unsigned size);

typedef void (*MainMemCb)(void* opaque,uint64_t exec,
		          int is_write,
		          void* phys,
				  uint64_t virt,
				  uint64_t size);
typedef void (*MainMemCbInternal)(
		          int is_write,
		          void* phys,
				  uint64_t virt,
				  uint64_t size);
typedef uint64_t (*ICacheMissCb)(void* opaque, uint64_t addr, unsigned size, int* tb_hit);
typedef void (*AddVictimCb)(void*);
typedef void(*FillBiasCb)(uint64_t* ts, int n);
extern MainMemCbInternal qslave_mem_notify;
extern MainMemCb qslave_mem_notify_model;
extern ICacheMissCb qslave_icache_miss_cb;
extern AddVictimCb qslave_add_victim_cb;
extern int qslave_counter_enable;

void qslave_icache_miss(uint64_t addr, unsigned size, int* tb_hit_flag);

enum OuterStat {
	L1_MISS=0,
	L2_MISS,
	L1_WB,
	L2_WB,
	L1_LD,
	L1_ST,
	L2_LD,
	L2_ST
};

typedef uint64_t (*OuterStatGetter)(int index, enum OuterStat type);

typedef void (*IOAccessCb)( uint32_t device,
                            uint64_t exec,
                            int is_write,
                            void* phys,
                            uint64_t virt,
                            uint64_t size,
                            uint64_t tag);
typedef void (*IOAccessCbInternal)(
                            uint32_t device,
                            int is_write,
                            void* phys,
                            uint64_t virt,
                            uint64_t size,
                            uint64_t tag);
extern IOAccessCbInternal qslave_ioaccess_notify;
extern IOAccessCb qslave_ioaccess_notify_model;

enum IOAccessStat {
    IOACCESS_READ=0,
    IOACCESS_WRITE
};
typedef uint64_t (*IOAccessStatGetter)(uint32_t device, enum IOAccessStat type);
typedef char(*IOAccessGetDelayCb)(uint32_t device, uint64_t* time_stamp, uint64_t* delay, uint64_t* tag);

typedef void (*SyncCb)(void* opaque, uint64_t executed, int wfi);
#pragma GCC visibility push(default)

void modelprovider_register_icache_miss_cb(ICacheMissCb cb);

void modelprovider_register_fill_bias_cb(FillBiasCb cb);
int modelprovider_configure(int argc, char **argv, char **envp);
void modelprovider_set_default_read_callback(ReadCb cb);
void modelprovider_set_default_write_callback(WriteCb cb);
void modelprovider_set_sync_callback(SyncCb cb);
uint64_t modelprovider_get_start_pc(int index);
void modelprovider_register_main_mem_callback(MainMemCb cb);

void modelprovider_register_outer_stat_cb(OuterStatGetter cb);

void modelprovider_register_ioaccess_callback(IOAccessCb cb);
void modelprovider_register_ioaccess_stat_cb(IOAccessStatGetter cb);
void modelprovider_register_ioaccess_get_delay_cb(IOAccessGetDelayCb cb);

void qslave_yield(void* opaque, uint64_t executed, int wfi);

uint64_t modelprovider_read_default(void *opaque,
                     hwaddr addr,
                     unsigned size) ;
void modelprovider_write_default(void *opaque,
                  hwaddr addr,
                  uint64_t data,
                  unsigned size);
void* modelprovider_create_internal_dev_default(char* name,
										uint64_t base,
										int irq,
										ReadCb* rd,
										WriteCb* wr);
void modelprovider_declare_external_dev(char* name, uint64_t base, uint64_t size);
void modelprovider_declare_external_ram(char* name, uint64_t base, uint64_t size, void* data);
void* modelprovider_create_internal_cpu(void *proxy, char* type, int index, uint64_t start_pc, int secure, int start_off);

void modelprovider_run_cpu(CPUState* cpu, uint64_t quantum);
void modelprovider_poll_io(void);

void modelprovider_finalize_config(void);

int modelprovider_thread_id(void);
void modelprovider_post_init(MachineState *machine);

void modelprovider_unlock(void);
void modelprovider_wait_unlock(void);

void modelprovider_register_unlock(void(*cb)(void*),void* mp);
void modelprovider_register_wait_unlock(void(*cb)(void*),void* mp);

void modelprovider_interrupt(int index, int value);

void modelprovider_cpu_register_stats(int index);

void modelprovider_show_cpu(void* handle);

void modelprovider_cpu_get_stats(int index, uint32_t*, void**);

void modelprovider_register_add_victim_cb(AddVictimCb cb);
#pragma GCC visibility pop

typedef struct {
	char name[512];
	uint64_t v;
} qslave_stat_entry;
qslave_stat_entry* qslave_cpu_get_stat(int index, char* name);

#define QSLAVE_STAT(index,nameS) qslave_cpu_stat _st##nameS; \
 strcpy(_st##nameS.nameS.name, #nameS); _st##nameS.nameS.v=0; \
 memcpy(&qslave_stat_cpu[index].nameS,&_st##nameS.nameS,sizeof(qslave_stat_entry))

typedef struct {
    qslave_stat_entry executed_instructions;
    qslave_stat_entry executed_fp_instructions;
    qslave_stat_entry executed_sve_instructions;
    qslave_stat_entry stores;
    qslave_stat_entry loads;
    qslave_stat_entry load_store;
    qslave_stat_entry count_tlb_hit;
    qslave_stat_entry count_tlb_miss;
} qslave_cpu_stat;

extern qslave_cpu_stat qslave_stat_cpu[MAX_CPUS];

// Used by PMU
uint64_t qslave_get_inst_count(int index);
uint64_t qslave_get_l1_miss(int index);
uint64_t qslave_get_l2_miss(int index);
uint64_t qslave_get_l1_wb(int index);
uint64_t qslave_get_l2_wb(int index);
uint64_t qslave_get_ld(int index);
uint64_t qslave_get_st(int index);
uint64_t qslave_get_l1_ld(int index);
uint64_t qslave_get_l1_st(int index);
uint64_t qslave_get_l2_ld(int index);
uint64_t qslave_get_l2_st(int index);

uint64_t qslave_get_icache_misses(int index);

void qslave_fill_biases(uint64_t* ts, int n);

uint64_t qslave_get_ioaccess_read(uint32_t device);
uint64_t qslave_get_ioaccess_write(uint32_t device);
char qslave_ioaccess_get_delay(uint32_t device, uint64_t* time_stamp, uint64_t* delay, uint64_t* tag);
void qslave_ioaccess_busy_get_delay(uint32_t device, uint64_t* time_stamp, uint64_t* delay, uint64_t* tag);

#endif /* _QSLAVE_H_ */
