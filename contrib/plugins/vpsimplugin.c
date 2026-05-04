/*
 * @file  vpsimplugin.c
 *
 * @authors  Mohamed Benazouz
 *
 * @version  1.0
 *
 * @brief  VPSim plugin for memory accesses notification through qslave interface.
 *
 * Copyright © 2026 CEA, LIST - All rights reserved.
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.1 or later.
 * See the COPYING.LIB file in the top-level directory.
 *
 * Software is distributed by CEA on an "AS IS" basis, WITHOUT WARRANTY OF
 * ANY KIND either express or implied.
 */

#include <qemu-plugin.h>
#include "exec/mmu-access-type.h"


QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

static void vcpu_mem_read(unsigned int cpu_index, qemu_plugin_meminfo_t meminfo,
                     uint64_t vaddr, void *udata)
{
    struct qemu_plugin_hwaddr *hwaddr;
    hwaddr = qemu_plugin_get_hwaddr(meminfo, vaddr);
    if (!qemu_plugin_hwaddr_is_io(hwaddr)) {
        qemu_plugin_mem_notify(cpu_index, MMU_DATA_LOAD, hwaddr, vaddr, 1 << qemu_plugin_mem_size_shift(meminfo));
    }
}

static void vcpu_mem_write(unsigned int cpu_index, qemu_plugin_meminfo_t meminfo,
                     uint64_t vaddr, void *udata)
{
    struct qemu_plugin_hwaddr *hwaddr;
    hwaddr = qemu_plugin_get_hwaddr(meminfo, vaddr);
    if (!qemu_plugin_hwaddr_is_io(hwaddr)) {
        qemu_plugin_mem_notify(cpu_index, MMU_DATA_STORE, hwaddr, vaddr, 1 << qemu_plugin_mem_size_shift(meminfo));
    }
}

static void vcpu_tb_trans(qemu_plugin_id_t id, struct qemu_plugin_tb *tb)
{
    size_t n = qemu_plugin_tb_n_insns(tb);
    size_t i;

    for (i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        qemu_plugin_register_vcpu_mem_cb(insn, vcpu_mem_read,
                                            QEMU_PLUGIN_CB_NO_REGS,
                                            QEMU_PLUGIN_MEM_R, NULL);
        qemu_plugin_register_vcpu_mem_cb(insn, vcpu_mem_write,
                                            QEMU_PLUGIN_CB_NO_REGS,
                                            QEMU_PLUGIN_MEM_W, NULL);
    }
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv)
{
    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans);
    return 0;
}
