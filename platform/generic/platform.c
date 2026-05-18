/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Anup Patel <anup.patel@wdc.com>
 */

#include <libfdt.h>
#include <platform_override.h>
#include <sbi/riscv_asm.h>
#include <sbi/sbi_bitops.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_hartmask.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_hsm.h>
#include <sbi/sbi_ipi.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_string.h>
#include <sbi/sbi_system.h>
#include <sbi/sbi_tlb.h>
#include <sbi_utils/cache/fdt_cmo_helper.h>
#include <sbi_utils/fdt/fdt_domain.h>
#include <sbi_utils/fdt/fdt_driver.h>
#include <sbi_utils/fdt/fdt_fixup.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/fdt/fdt_pmu.h>
#include <sbi_utils/irqchip/fdt_irqchip.h>
#include <sbi_utils/irqchip/imsic.h>
#include <sbi_utils/mpxy/fdt_mpxy.h>
#include <sbi_utils/serial/fdt_serial.h>
#include <sbi_utils/serial/semihosting.h>
#include <sbi_utils/timer/fdt_timer.h>
#include <sbi_utils/serial/virtual-uart.h>

/* List of platform override modules generated at compile time */
extern const struct fdt_driver *const platform_override_modules[];

static u32 fw_platform_calculate_heap_size(u32 hart_count)
{
	u32 heap_size;

	heap_size = SBI_PLATFORM_DEFAULT_HEAP_SIZE(hart_count);

	/* For TLB fifo */
	heap_size += SBI_TLB_INFO_SIZE * (hart_count) * (hart_count);

	return BIT_ALIGN(heap_size, HEAP_BASE_ALIGN);
}

static u32 fw_platform_get_heap_size(const void *fdt, u32 hart_count)
{
	int chosen_offset, config_offset, len;
	const fdt32_t *val;

	/* Get the heap size from device tree */
	chosen_offset = fdt_path_offset(fdt, "/chosen");
	if (chosen_offset < 0)
		goto default_config;

	config_offset = fdt_node_offset_by_compatible(fdt, chosen_offset, "opensbi,config");
	if (config_offset < 0)
		goto default_config;

	val = (fdt32_t *)fdt_getprop(fdt, config_offset, "heap-size", &len);
	if (len > 0 && val)
		return BIT_ALIGN(fdt32_to_cpu(*val), HEAP_BASE_ALIGN);

default_config:
	return fw_platform_calculate_heap_size(hart_count);
}

extern struct sbi_platform platform;
static bool platform_has_mlevel_imsic = false;
static u32 generic_hart_index2id[SBI_HARTMASK_MAX_BITS] = { 0 };

static DECLARE_BITMAP(generic_coldboot_harts, SBI_HARTMASK_MAX_BITS);

/*
 * The fw_platform_coldboot_harts_init() function is called by fw_platform_init()
 * function to initialize the cold boot harts allowed by the generic platform
 * according to the DT property "cold-boot-harts" in "/chosen/opensbi-config"
 * DT node. If there is no "cold-boot-harts" in DT, all harts will be allowed.
 */
static void fw_platform_coldboot_harts_init(const void *fdt)
{
	int chosen_offset, config_offset, cpu_offset, len, err;
	u32 val32;
	const u32 *val;

	bitmap_zero(generic_coldboot_harts, SBI_HARTMASK_MAX_BITS);

	chosen_offset = fdt_path_offset(fdt, "/chosen");
	if (chosen_offset < 0)
		goto default_config;

	config_offset = fdt_node_offset_by_compatible(fdt, chosen_offset, "opensbi,config");
	if (config_offset < 0)
		goto default_config;

	val = fdt_getprop(fdt, config_offset, "cold-boot-harts", &len);
	if (!val || !len)
		goto default_config;

	len = len / sizeof(u32);
	for (int i = 0; i < len; i++) {
		cpu_offset = fdt_node_offset_by_phandle(fdt,
						fdt32_to_cpu(val[i]));
		if (cpu_offset < 0)
			goto default_config;

		err = fdt_parse_hart_id(fdt, cpu_offset, &val32);
		if (err)
			goto default_config;

		if (!fdt_node_is_enabled(fdt, cpu_offset))
			continue;

		for (int i = 0; i < platform.hart_count; i++) {
			if (val32 == generic_hart_index2id[i])
				bitmap_set(generic_coldboot_harts, i, 1);
		}
	}

	return;

default_config:
	bitmap_fill(generic_coldboot_harts, SBI_HARTMASK_MAX_BITS);
	return;
}

/*
 * The fw_platform_init() function is called very early on the boot HART
 * OpenSBI reference firmwares so that platform specific code get chance
 * to update "platform" instance before it is used.
 *
 * The arguments passed to fw_platform_init() function are boot time state
 * of A0 to A4 register. The "arg0" will be boot HART id and "arg1" will
 * be address of FDT passed by previous booting stage.
 *
 * The return value of fw_platform_init() function is the FDT location. If
 * FDT is unchanged (or FDT is modified in-place) then fw_platform_init()
 * can always return the original FDT location (i.e. 'arg1') unmodified.
 */
unsigned long fw_platform_init(unsigned long arg0, unsigned long arg1,
				unsigned long arg2, unsigned long arg3,
				unsigned long arg4)
{
	const char *model;
	const void *fdt = (void *)arg1;
	u32 hartid, hart_count = 0;
	int rc, root_offset, cpus_offset, cpu_offset, len;
	unsigned long cbom_block_size = 0;
	unsigned long tmp = 0;

	root_offset = fdt_path_offset(fdt, "/");
	if (root_offset < 0)
		goto fail;

	fdt_driver_init_by_offset(fdt, root_offset, platform_override_modules);

	model = fdt_getprop(fdt, root_offset, "model", &len);
	if (model)
		sbi_strncpy(platform.name, model, sizeof(platform.name) - 1);

	cpus_offset = fdt_path_offset(fdt, "/cpus");
	if (cpus_offset < 0)
		goto fail;

	fdt_for_each_subnode(cpu_offset, fdt, cpus_offset) {
		rc = fdt_parse_hart_id(fdt, cpu_offset, &hartid);
		if (rc)
			continue;

		if (SBI_HARTMASK_MAX_BITS <= hart_count)
			break;

		if (!fdt_node_is_enabled(fdt, cpu_offset))
			continue;

		generic_hart_index2id[hart_count++] = hartid;

		rc = fdt_parse_cbom_block_size(fdt, cpu_offset, &tmp);
		if (rc)
			continue;
		cbom_block_size = MAX(tmp, cbom_block_size);
	}

	platform.hart_count = hart_count;
	platform.heap_size = fw_platform_get_heap_size(fdt, hart_count);
	platform_has_mlevel_imsic = fdt_check_imsic_mlevel(fdt);
	platform.cbom_block_size = cbom_block_size;

	fw_platform_coldboot_harts_init(fdt);

	/* Return original FDT pointer */
	return arg1;

fail:
	while (1)
		wfi();
}

bool generic_cold_boot_allowed(u32 hartid)
{
	for (int i = 0; i < platform.hart_count; i++) {
		if (hartid == generic_hart_index2id[i])
			return bitmap_test(generic_coldboot_harts, i);
	}

	return false;
}

int generic_nascent_init(void)
{
	if (platform_has_mlevel_imsic)
		imsic_local_irqchip_init();
	return 0;
}

int generic_early_init(bool cold_boot)
{
	const void *fdt = fdt_get_address();
	int rc;

	if (cold_boot) {
		if (semihosting_enabled())
			rc = semihosting_init();
		else
			rc = fdt_serial_init(fdt);
		if (rc == SBI_ENODEV)
			rc = virtual_uart_init();
		if (rc)
			return rc;

		fdt_driver_init_all(fdt, fdt_early_drivers);
	}

	return fdt_cmo_init(cold_boot);
}

/* bhx#166 Phase 1: forward decl, definition is later in this file. */
static void bhx_purgatory_register_reset_device(void);
/* bhx#166 Phase 5: forward decl, definition is later in this file. */
static void bhx_purgatory_register_force_park_event(void);
static void bhx_purgatory_publish_force_park_metadata(void);

int generic_final_init(bool cold_boot)
{
	void *fdt = fdt_get_address_rw();

	if (!cold_boot)
		return 0;

	fdt_cpu_fixup(fdt);
	fdt_fixups(fdt);
	fdt_domain_fixup(fdt);

	/* Set the empty space in FDT based on kconfig option */
	fdt_pack(fdt);
	fdt_open_into(fdt, fdt, fdt_totalsize(fdt) +
		      CONFIG_PLATFORM_GENERIC_FDT_EMPTY_SPACE * 1024);

	/* bhx#166 Phase 1: register stub reset device so SBI SRST → sbi_exit. */
	bhx_purgatory_register_reset_device();

	/* bhx#166 Phase 5: register the force-park IPI event and publish
	 * its metadata in the purgatory status block. Done at cold init so
	 * the host can fire force-park on a Running guest before any SRST
	 * has happened — final_exit only runs ON SRST. */
	bhx_purgatory_register_force_park_event();
	bhx_purgatory_publish_force_park_metadata();

	return 0;
}

int generic_extensions_init(struct sbi_hart_features *hfeatures)
{
	/* Parse the ISA string from FDT and enable the listed extensions */
	return fdt_parse_isa_extensions(fdt_get_address(), current_hartid(),
					hfeatures->extensions);
}

int generic_domains_init(void)
{
	const void *fdt = fdt_get_address();
	int offset, ret;

	ret = fdt_domains_populate(fdt);
	if (ret < 0)
		return ret;

	offset = fdt_path_offset(fdt, "/chosen");

	if (offset >= 0) {
		offset = fdt_node_offset_by_compatible(fdt, offset,
						       "opensbi,config");
		if (offset >= 0 &&
		    fdt_get_property(fdt, offset, "system-suspend-test", NULL))
			sbi_system_suspend_test_enable();
	}

	return 0;
}

u64 generic_tlbr_flush_limit(void)
{
	return SBI_PLATFORM_TLB_RANGE_FLUSH_LIMIT_DEFAULT;
}

u32 generic_tlb_num_entries(void)
{
	return sbi_hart_count();
}

int generic_pmu_init(void)
{
	int rc;

	rc = fdt_pmu_setup(fdt_get_address());
	if (rc && rc != SBI_ENOENT)
		return rc;

	return 0;
}

uint64_t generic_pmu_xlate_to_mhpmevent(uint32_t event_idx, uint64_t data)
{
	uint64_t evt_val = 0;

	/* data is valid only for raw events and is equal to event selector */
	if (event_idx == SBI_PMU_EVENT_RAW_IDX ||
		event_idx == SBI_PMU_EVENT_RAW_V2_IDX)
		evt_val = data;
	else {
		/**
		 * Generic platform follows the SBI specification recommendation
		 * i.e. zero extended event_idx is used as mhpmevent value for
		 * hardware general/cache events if platform does't define one.
		 */
		evt_val = fdt_pmu_get_select_value(event_idx);
		if (!evt_val)
			evt_val = (uint64_t)event_idx;
	}

	return evt_val;
}

int generic_mpxy_init(void)
{
	const void *fdt = fdt_get_address();

	return fdt_mpxy_init(fdt);
}

/* bhx#166 Phase 1+2+4a: announce parked-in-warmboot to the PCIe host
 * after multi-hart convergence, and publish the metadata the host
 * needs to release hart 0 from the HSM warmboot wait loop.
 *
 * Called from sbi_exit() right before sbi_hsm_exit() jumps into the
 * HSM warmboot wait loop. The SRST-issuing hart broadcasts halt-IPIs
 * before reaching here; peers walk sbi_hsm_hart_stop -> sbi_hsm_exit
 * (which atomically transitions STOP_PENDING -> STOPPED) and join the
 * warmboot wait loop on their own. We poll their HSM state until
 * every peer reaches STOPPED, write the metadata + peers bitmask,
 * then write the PARKED magic last (single-writer fence so the host
 * never observes PARKED with stale metadata). The host therefore
 * sees PARKED only when every hart in the tile is quiesced AND the
 * release metadata is current.
 *
 * State quiesce on the SRST-issuing hart (clearing pending IPIs,
 * timer, external IRQs) is already done by sbi_exit's earlier calls
 * to sbi_timer_exit / sbi_ipi_exit / sbi_irqchip_exit; we don't
 * duplicate that here. Cache flush + PMP re-validation are deferred
 * to the host (see #166).
 *
 * Status block layout (all u64 little-endian; the SRST-issuing hart
 * writes status_word LAST so the host can poll the magic and trust
 * the rest of the block once the magic shows up):
 *
 *   +0x00 : status word
 *           0x0000_0000_0000_0000 = not yet parked (cold-boot zero)
 *           0x5f5f_4445_4b52_4150 = "PARKED__" magic
 *
 *   +0x08 : peer convergence bitmask. Low N bits = harts 0..N-1 that
 *           reached SBI_HSM_STATE_STOPPED before the SRST-issuing
 *           hart announced PARKED. The SRST-issuing hart's bit is
 *           NOT set (it transitions itself in sbi_hsm_exit AFTER
 *           final_exit returns). Full convergence on a 4-hart tile
 *           when hart 0 issued SRST: 0xE.
 *
 *   +0x10 : hart 0's `&scratch->next_addr` PA. Host writes the new
 *           kernel entry-point here.
 *
 *   +0x18 : hart 0's `&scratch->next_mode` PA. Host writes
 *           PRV_S = 1 (next mode after sbi_hart_switch_mode).
 *
 *   +0x20 : hart 0's `&scratch->next_arg1` PA. Host writes the new
 *           DTB PA (becomes a1 on entry to the kernel).
 *
 *   +0x28 : hart 0's HSM state PA. Host writes
 *           SBI_HSM_STATE_START_PENDING = 2 to wake hart 0 from
 *           sbi_hsm_hart_wait. Read STOPPED = 1 first to confirm
 *           the hart hasn't been started by some other path.
 *
 *   +0x30 : CLINT MSIP[0] PA. Host writes 1 to fire the M-mode
 *           software interrupt that wakes hart 0 from `wfi`.
 *
 *   +0x48 : reset_type (u64; low 32 bits carry SBI_SRST_RESET_TYPE_*).
 *           0 = SHUTDOWN  (guest `poweroff` / `init 0` / force-park IPI)
 *           1 = COLD_REBOOT
 *           2 = WARM_REBOOT
 *           Stashed by `bhx_purgatory_reset_do` (which sees the type
 *           passed by sbi_system_reset) and published here so the host
 *           can distinguish "fast-resume the same guest" from "guest
 *           said it's done; release disk + net resources" (bhx#177).
 *
 * Metadata is computed at first call. Subsequent SRSTs reuse the
 * already-published values (they don't change across re-entries
 * because OpenSBI's bss is preserved through the HSM warmboot loop).
 * reset_type is re-stashed on every SRST so the host always sees the
 * type of the *most recent* shutdown when it observes PARKED.
 */
#define BHX_PURGATORY_STATUS_OFFSET	0x000E0000UL
#define BHX_PURGATORY_STATUS_PARKED	0x5f5f44454b524150UL  /* "PARKED__" LE */
#define BHX_PURGATORY_PEERS_OFFSET	(BHX_PURGATORY_STATUS_OFFSET + 8)
#define BHX_PURGATORY_NEXT_ADDR_OFFSET	(BHX_PURGATORY_STATUS_OFFSET + 0x10)
#define BHX_PURGATORY_NEXT_MODE_OFFSET	(BHX_PURGATORY_STATUS_OFFSET + 0x18)
#define BHX_PURGATORY_NEXT_ARG1_OFFSET	(BHX_PURGATORY_STATUS_OFFSET + 0x20)
#define BHX_PURGATORY_HSM_STATE_OFFSET	(BHX_PURGATORY_STATUS_OFFSET + 0x28)
#define BHX_PURGATORY_MSIP_OFFSET	(BHX_PURGATORY_STATUS_OFFSET + 0x30)
/* (#166 Phase 5) Force-park IPI metadata. The host writes
 * `force_park_request_value` to the address at
 * `force_park_request_pa` (an `unsigned long *` pointing at hart 0's
 * `&ipi_data->ipi_type`), then writes 1 to MSIP. OpenSBI's existing
 * IPI dispatcher routes via the registered `bhx_force_park` event's
 * `process` callback, which calls `sbi_system_reset` and walks the
 * same final_exit + warmboot path the SBI SRST case uses. */
#define BHX_PURGATORY_FORCE_PARK_REQ_PA_OFFSET		(BHX_PURGATORY_STATUS_OFFSET + 0x38)
#define BHX_PURGATORY_FORCE_PARK_REQ_VALUE_OFFSET	(BHX_PURGATORY_STATUS_OFFSET + 0x40)
/* (bhx#177) reset_type stashed by `bhx_purgatory_reset_do` and
 * published by `bhx_purgatory_final_exit`. Lets the host distinguish
 * SHUTDOWN (drop disk+net workers) from REBOOT (keep them for fast
 * resume). */
#define BHX_PURGATORY_RESET_TYPE_OFFSET			(BHX_PURGATORY_STATUS_OFFSET + 0x48)

/* L2CPU local CLINT base — see third_party/dtb/blackhole.dtsi
 * (`tenstorrent,blackhole-clint` / `sifive,clint0` at 0x2000000).
 * Host accesses it via the per-L2CPU TLB window that already covers
 * [0, 4 GiB) of the L2CPU's local bus. */
#define TT_L2CPU_CLINT_BASE		0x02000000UL
#define TT_L2CPU_CLINT_MSIP(hart)	(TT_L2CPU_CLINT_BASE + 4UL * (hart))

/* Maximum spin iterations waiting for peers to reach STOPPED. At
 * 1750 MHz this caps the wait around ~10ms (single inline pause
 * per iter), well above the ~µs latency of an in-tile IPI dispatch
 * but short enough that a wedged hart fails fast rather than hanging
 * the SRST path. */
#define BHX_PURGATORY_PEER_WAIT_ITERS	(64u * 1024u)

static unsigned long bhx_purgatory_collect_peers_stopped(u32 self_hartindex)
{
	unsigned long mask = 0;
	u32 i, count = sbi_hart_count();

	for (i = 0; i < count && i < 64; i++) {
		if (i == self_hartindex)
			continue;
		if (__sbi_hsm_hart_get_state(i) == SBI_HSM_STATE_STOPPED)
			mask |= 1UL << i;
	}
	return mask;
}

/* (bhx#177) Last SBI_SRST_RESET_TYPE_* passed to
 * `bhx_purgatory_reset_do` for the current SRST cycle. BSS-initialized
 * to 0 (= SHUTDOWN), which is the safe default if final_exit were to
 * somehow fire without reset_do being called first — host treats it
 * as "drop disk+net" rather than the more leaky "keep them around."
 * In practice reset_do is always called before final_exit because
 * final_exit is invoked from sbi_exit() which is called from
 * sbi_system_reset() which is what calls reset_do. */
static u32 bhx_last_reset_type;

static void bhx_purgatory_final_exit(void)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	struct sbi_scratch *hart0_scratch = sbi_hartindex_to_scratch(0);
	void *hart0_state = sbi_hsm_hart_state_addr(0);
	u32 hartindex = current_hartindex();
	u32 hart_count = sbi_hart_count();
	unsigned long expected_mask = 0, peers_mask = 0;
	unsigned int spin = 0;
	u32 i;
	volatile unsigned long *status =
		(volatile unsigned long *)(scratch->fw_start +
					   BHX_PURGATORY_STATUS_OFFSET);
	volatile unsigned long *peers =
		(volatile unsigned long *)(scratch->fw_start +
					   BHX_PURGATORY_PEERS_OFFSET);
	volatile unsigned long *next_addr_pa =
		(volatile unsigned long *)(scratch->fw_start +
					   BHX_PURGATORY_NEXT_ADDR_OFFSET);
	volatile unsigned long *next_mode_pa =
		(volatile unsigned long *)(scratch->fw_start +
					   BHX_PURGATORY_NEXT_MODE_OFFSET);
	volatile unsigned long *next_arg1_pa =
		(volatile unsigned long *)(scratch->fw_start +
					   BHX_PURGATORY_NEXT_ARG1_OFFSET);
	volatile unsigned long *hsm_state_pa =
		(volatile unsigned long *)(scratch->fw_start +
					   BHX_PURGATORY_HSM_STATE_OFFSET);
	volatile unsigned long *msip_pa =
		(volatile unsigned long *)(scratch->fw_start +
					   BHX_PURGATORY_MSIP_OFFSET);
	volatile unsigned long *reset_type_pa =
		(volatile unsigned long *)(scratch->fw_start +
					   BHX_PURGATORY_RESET_TYPE_OFFSET);

	/* Build the expected peer mask: every hart except ourselves. */
	for (i = 0; i < hart_count && i < 64; i++)
		if (i != hartindex)
			expected_mask |= 1UL << i;

	/* Wait for peers to reach STOPPED. */
	while (spin++ < BHX_PURGATORY_PEER_WAIT_ITERS) {
		peers_mask = bhx_purgatory_collect_peers_stopped(hartindex);
		if (peers_mask == expected_mask)
			break;
	}

	/* Phase 4a: publish hart 0's release metadata. */
	*next_addr_pa = hart0_scratch ?
		(unsigned long)&hart0_scratch->next_addr : 0UL;
	*next_mode_pa = hart0_scratch ?
		(unsigned long)&hart0_scratch->next_mode : 0UL;
	*next_arg1_pa = hart0_scratch ?
		(unsigned long)&hart0_scratch->next_arg1 : 0UL;
	*hsm_state_pa = (unsigned long)hart0_state;
	*msip_pa = TT_L2CPU_CLINT_MSIP(0);
	/* (bhx#177) Publish the SRST type the guest issued so the host
	 * can decide whether to release disk+net resources (SHUTDOWN) or
	 * keep them warm for fast resume (REBOOT). */
	*reset_type_pa = (unsigned long)bhx_last_reset_type;

	*peers = peers_mask;
	__asm__ volatile("fence ow,ow" : : : "memory");
	*status = BHX_PURGATORY_STATUS_PARKED;
	__asm__ volatile("fence ow,ow" : : : "memory");
}

/* Stub reset device so the kernel sees SBI SRST as supported. Returns
 * priority 1 (same as syscon-poweroff). Registered in final_init AFTER
 * fdt_reset_init has scanned for syscon, so on a tie syscon wins; we
 * only become the active reset device when no other one was registered.
 * system_reset() returns immediately, letting sbi_system_reset() fall
 * through to sbi_exit() and ultimately our final_exit hook.
 */
static int bhx_purgatory_reset_check(u32 reset_type, u32 reset_reason)
{
	(void)reset_reason;
	if (reset_type == SBI_SRST_RESET_TYPE_SHUTDOWN ||
	    reset_type == SBI_SRST_RESET_TYPE_COLD_REBOOT ||
	    reset_type == SBI_SRST_RESET_TYPE_WARM_REBOOT)
		return 1;
	return 0;
}

static void bhx_purgatory_reset_do(u32 reset_type, u32 reset_reason)
{
	(void)reset_reason;
	/* (bhx#177) Stash the type for `bhx_purgatory_final_exit` to
	 * publish into the status block. final_exit runs after this
	 * (sbi_system_reset → reset_do → sbi_exit → final_exit), so a
	 * single static is the right shape: written here, read there,
	 * no concurrency. */
	bhx_last_reset_type = reset_type;
	/* No-op past the stash: caller's sbi_system_reset() falls
	 * through to sbi_exit() and onward through final_exit. */
}

static struct sbi_system_reset_device bhx_purgatory_reset_device = {
	.name = "bhx-purgatory",
	.system_reset_check = bhx_purgatory_reset_check,
	.system_reset = bhx_purgatory_reset_do,
};

static void bhx_purgatory_register_reset_device(void)
{
	sbi_system_reset_add_device(&bhx_purgatory_reset_device);
}

/* bhx#166 Phase 5: force-park IPI event. The PCIe host fires this
 * to take any running guest into the same parked state a SBI SRST
 * would have produced — recovery for kernels wedged in S-mode with
 * `sstatus.SIE=0` (M-mode IPIs preempt unconditionally). The host's
 * sequence is:
 *
 *   1. Read FORCE_PARK_REQ_PA + FORCE_PARK_REQ_VALUE from the
 *      purgatory status block (published once at cold init).
 *   2. Write the value to the address. This sets the event-pending
 *      bit in hart 0's `sbi_ipi_data->ipi_type`. No concurrent IPI
 *      sender exists (the kernel can't issue M-mode IPIs), so a
 *      direct u64 write is equivalent to atomic-OR.
 *   3. Write 1 to MSIP_PA (CLINT MSIP[0]) to deliver the M-mode
 *      software interrupt to hart 0.
 *
 * Hart 0 takes the trap, OpenSBI's IPI dispatcher walks ipi_type
 * and invokes our `process` callback, which calls sbi_system_reset
 * — same path as a guest-issued SBI SRST. peers get the halt-IPI,
 * everyone walks through final_exit + warmboot, host sees PARKED
 * magic, drives release-from-purgatory.
 */
static void bhx_force_park_process(struct sbi_scratch *scratch)
{
	(void)scratch;
	sbi_system_reset(SBI_SRST_RESET_TYPE_SHUTDOWN, 0);
	/* sbi_system_reset is __noreturn — it broadcasts halt-IPI to
	 * peers, stops self, walks sbi_exit → final_exit → jump_warmboot.
	 * If we ever return here it means the IPI dispatcher's contract
	 * changed; sbi_hart_hang is the safest thing to do. */
	__builtin_unreachable();
}

static const struct sbi_ipi_event_ops bhx_force_park_ops = {
	.name = "BHX_FORCE_PARK",
	.process = bhx_force_park_process,
};

static int bhx_force_park_event = -1;

static void bhx_purgatory_register_force_park_event(void)
{
	int ev = sbi_ipi_event_create(&bhx_force_park_ops);
	if (ev < 0)
		return;  /* Out of event slots — host force-park unavailable. */
	bhx_force_park_event = ev;
}

static void bhx_purgatory_publish_force_park_metadata(void)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	volatile unsigned long *req_pa =
		(volatile unsigned long *)(scratch->fw_start +
				BHX_PURGATORY_FORCE_PARK_REQ_PA_OFFSET);
	volatile unsigned long *req_value =
		(volatile unsigned long *)(scratch->fw_start +
				BHX_PURGATORY_FORCE_PARK_REQ_VALUE_OFFSET);
	/* The MSIP PA is needed at cold init for Phase 5 (force-park
	 * fires before any SRST), separate from the Phase 4a
	 * publication in final_exit (only valid post-SRST). Same value
	 * either way — CLINT MSIP[0] is statically defined. Publishing
	 * here makes it available immediately after cold init. */
	volatile unsigned long *msip_pa =
		(volatile unsigned long *)(scratch->fw_start +
				BHX_PURGATORY_MSIP_OFFSET);
	unsigned long *ipi_type_pa = sbi_ipi_data_addr(0);

	if (bhx_force_park_event < 0 || !ipi_type_pa) {
		*req_pa = 0;
		*req_value = 0;
	} else {
		*req_pa = (unsigned long)ipi_type_pa;
		*req_value = 1UL << bhx_force_park_event;
	}
	*msip_pa = TT_L2CPU_CLINT_MSIP(0);
	__asm__ volatile("fence ow,ow" : : : "memory");
}


struct sbi_platform_operations generic_platform_ops = {
	.cold_boot_allowed	= generic_cold_boot_allowed,
	.nascent_init		= generic_nascent_init,
	.early_init		= generic_early_init,
	.final_init		= generic_final_init,
	.extensions_init	= generic_extensions_init,
	.domains_init		= generic_domains_init,
	.irqchip_init		= fdt_irqchip_init,
	.pmu_init		= generic_pmu_init,
	.pmu_xlate_to_mhpmevent = generic_pmu_xlate_to_mhpmevent,
	.get_tlbr_flush_limit	= generic_tlbr_flush_limit,
	.get_tlb_num_entries	= generic_tlb_num_entries,
	.timer_init		= fdt_timer_init,
	.mpxy_init		= generic_mpxy_init,
	.final_exit		= bhx_purgatory_final_exit,
};

struct sbi_platform platform = {
	.opensbi_version	= OPENSBI_VERSION,
	.platform_version	=
		SBI_PLATFORM_VERSION(CONFIG_PLATFORM_GENERIC_MAJOR_VER,
				     CONFIG_PLATFORM_GENERIC_MINOR_VER),
	.name			= CONFIG_PLATFORM_GENERIC_NAME,
	.features		= SBI_PLATFORM_DEFAULT_FEATURES,
	.hart_count		= SBI_HARTMASK_MAX_BITS,
	.hart_index2id		= generic_hart_index2id,
	.hart_stack_size	= SBI_PLATFORM_DEFAULT_HART_STACK_SIZE,
	.heap_size		= SBI_PLATFORM_DEFAULT_HEAP_SIZE(0),
	.platform_ops_addr	= (unsigned long)&generic_platform_ops
};
