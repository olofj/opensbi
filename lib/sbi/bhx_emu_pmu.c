// SPDX-License-Identifier: BSD-2-Clause
/*
 * bhx ISA-extension-emulation PMU device.
 *
 * Provides the bhx strong overrides of sbi_insn_emu_pmu_inc() and
 * sbi_insn_emu_pmu_init() from <sbi/sbi_insn_emu_pmu.h>. Counts
 * trap-handler invocations for each emulated extension (Zcb, Zba,
 * Zbb, Zbs, Zicond, Zimop, Zawrs, Zfa, Zfhmin, Zicbom, Zicboz,
 * Zcmop, Zbc, Supm) plus an aggregate "any emulated insn fired"
 * counter, and an UNHANDLED counter for traps that reached
 * truly_illegal_insn() without the emulator helping.
 *
 * Two read paths share one storage:
 *
 *  - Guest-side: SBI PMU platform firmware events (event_code =
 *    SBI_PMU_FW_PLATFORM, event_data = SBI_INSN_EMU_EXT_*). Linux
 *    perf with raw `config = 0xC000_0000_0000_0000 | event_id`
 *    routes here via riscv_pmu_sbi.c.
 *
 *  - Host-side: a statically-placed `bhx_emu_pmu_publish` struct
 *    that the bhx daemon reads over PCIe by following a pointer
 *    parked at fw_jump.bin + 0x88 (sibling to the debug_descriptor
 *    pointer at +0x80). No guest cooperation needed — useful when
 *    the guest is wedged or hasn't reached a perf-capable userspace.
 *
 * Per-hart counters live directly in the publish struct (BSS — zero
 * at boot). No sbi_scratch allocation needed.
 */

#include <sbi/bhx_emu_pmu_publish.h>
#include <sbi/riscv_asm.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_ecall_interface.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_insn_emu_pmu.h>
#include <sbi/sbi_pmu.h>
#include <sbi/sbi_string.h>

/*
 * The publish struct itself. Statically initialized with magic/
 * version so the host can validate the page contents without
 * trusting that init has run. Counters land in BSS (zero) and
 * accumulate from there.
 */
struct bhx_emu_pmu_publish bhx_emu_pmu_publish
__attribute__((aligned(64))) = {
	.magic		= BHX_EMU_PMU_MAGIC,
	.version	= BHX_EMU_PMU_VERSION,
	.max_harts	= BHX_EMU_PMU_MAX_HARTS,
	.max_events	= SBI_INSN_EMU_EXT_MAX,
};

/*
 * Strong override of the generic emu-PMU increment hook. Bumps the
 * per-(hart, ext) counter plus the per-hart ANY aggregate. NONE /
 * out-of-range is a silent no-op so emu call sites can pass an
 * un-classified match-tracker variable without branching.
 */
void sbi_insn_emu_pmu_inc(int ext)
{
	u32 hartid;

	if (ext <= 0 || ext >= SBI_INSN_EMU_EXT_MAX)
		return;
	hartid = current_hartid();
	if (hartid >= BHX_EMU_PMU_MAX_HARTS)
		return;
	/* Per-hart counter — no cross-hart access from the hot path,
	 * plain increment is fine. The read-side
	 * (fw_counter_read_value) runs in the SBI ecall context which
	 * may be on a different hart, but a torn 64-bit read on a
	 * counter that's actively incrementing is acceptable: perf
	 * tolerates it the same way it tolerates rdtsc skew. */
	bhx_emu_pmu_publish.counts[hartid][ext]++;
	/* ANY is bumped on every non-ANY inc — including UNHANDLED.
	 * The invariant `ANY - sum(handled_ext) == UNHANDLED` holds
	 * because UNHANDLED is a non-ANY ext that bumps itself once
	 * plus ANY once. Skipping the ANY bump when ext == ANY keeps
	 * a direct `sbi_insn_emu_pmu_inc(SBI_INSN_EMU_EXT_ANY)` call
	 * (none today, but possible) from double-counting. */
	if (ext != SBI_INSN_EMU_EXT_ANY)
		bhx_emu_pmu_publish.counts[hartid][SBI_INSN_EMU_EXT_ANY]++;
}

static int validate_encoding(uint32_t hartid, uint64_t event_data)
{
	(void)hartid;
	if (event_data >= 1 && event_data < SBI_INSN_EMU_EXT_MAX)
		return 0;
	return SBI_EINVAL;
}

static bool counter_match_encoding(uint32_t hartid, uint32_t ctr_idx,
				   uint64_t event_data)
{
	if (hartid >= BHX_EMU_PMU_MAX_HARTS || ctr_idx >= SBI_PMU_FW_CTR_MAX)
		return false;
	return bhx_emu_pmu_publish.ctr_to_event[hartid][ctr_idx]
		== (int)event_data;
}

static int counter_width(void)
{
	/* 63 = "64-bit counter, but only 63 bits visible" — matches
	 * what other SBI PMU platforms report. Counters never wrap
	 * in realistic timeframes. */
	return 63;
}

static uint64_t counter_read_value(uint32_t hartid, uint32_t ctr_idx)
{
	int event_id;

	if (hartid >= BHX_EMU_PMU_MAX_HARTS || ctr_idx >= SBI_PMU_FW_CTR_MAX)
		return 0;
	event_id = bhx_emu_pmu_publish.ctr_to_event[hartid][ctr_idx];
	if (event_id <= 0 || event_id >= SBI_INSN_EMU_EXT_MAX)
		return 0;
	return bhx_emu_pmu_publish.counts[hartid][event_id];
}

static void counter_write_value(uint32_t hartid, uint32_t ctr_idx,
				uint64_t value)
{
	int event_id;

	if (hartid >= BHX_EMU_PMU_MAX_HARTS || ctr_idx >= SBI_PMU_FW_CTR_MAX)
		return;
	event_id = bhx_emu_pmu_publish.ctr_to_event[hartid][ctr_idx];
	if (event_id <= 0 || event_id >= SBI_INSN_EMU_EXT_MAX)
		return;
	bhx_emu_pmu_publish.counts[hartid][event_id] = value;
}

static int counter_start(uint32_t hartid, uint32_t ctr_idx,
			 uint64_t event_data)
{
	if (hartid >= BHX_EMU_PMU_MAX_HARTS || ctr_idx >= SBI_PMU_FW_CTR_MAX)
		return SBI_EINVAL;
	if (event_data == 0 || event_data >= SBI_INSN_EMU_EXT_MAX)
		return SBI_EINVAL;
	bhx_emu_pmu_publish.ctr_to_event[hartid][ctr_idx] = (int)event_data;
	return 0;
}

static int counter_stop(uint32_t hartid, uint32_t ctr_idx)
{
	if (hartid >= BHX_EMU_PMU_MAX_HARTS || ctr_idx >= SBI_PMU_FW_CTR_MAX)
		return SBI_EINVAL;
	bhx_emu_pmu_publish.ctr_to_event[hartid][ctr_idx] = 0;
	return 0;
}

static const struct sbi_pmu_device bhx_emu_pmu = {
	.name				= "bhx-isa-emu",
	.fw_event_validate_encoding	= validate_encoding,
	.fw_counter_match_encoding	= counter_match_encoding,
	.fw_counter_width		= counter_width,
	.fw_counter_read_value		= counter_read_value,
	.fw_counter_write_value		= counter_write_value,
	.fw_counter_start		= counter_start,
	.fw_counter_stop		= counter_stop,
};

/*
 * Strong override of the generic sbi_insn_emu_pmu_init() hook.
 *
 * Zeros the current hart's slot of the publish struct and registers
 * the bhx PMU device with the SBI PMU core. The zeroing is
 * important on the *warm-reboot* path (guest reboot through
 * bhx-purgatory release): OpenSBI's _start_warm doesn't clear .bss
 * or reload .data, so without this each guest reboot would inherit
 * the previous boot's counts and first-unhandled capture. With it,
 * every boot — cold or warm — starts the per-boot stats clean.
 *
 * Cold init still works because the publish struct's header
 * (magic/version/max_harts/max_events) is statically initialized
 * in .data and never touched by this function. Only the per-hart
 * count + capture state moves.
 */
int sbi_insn_emu_pmu_init(void)
{
	u32 hartid = current_hartid();
	int i;

	if (hartid < BHX_EMU_PMU_MAX_HARTS) {
		for (i = 0; i < SBI_INSN_EMU_EXT_MAX; i++)
			bhx_emu_pmu_publish.counts[hartid][i] = 0;
		for (i = 0; i < SBI_PMU_FW_CTR_MAX; i++)
			bhx_emu_pmu_publish.ctr_to_event[hartid][i] = 0;
		bhx_emu_pmu_publish.first_unhandled_insn[hartid] = 0;
		bhx_emu_pmu_publish.first_unhandled_mepc[hartid] = 0;
		for (i = 0; i < BHX_EMU_PMU_UNHANDLED_TABLE_SIZE; i++) {
			bhx_emu_pmu_publish.unhandled_table[hartid][i].insn = 0;
			bhx_emu_pmu_publish.unhandled_table[hartid][i].first_mepc = 0;
			bhx_emu_pmu_publish.unhandled_table[hartid][i].count = 0;
		}
		bhx_emu_pmu_publish.unhandled_overflow[hartid] = 0;
	}
	sbi_pmu_set_device(&bhx_emu_pmu);
	return 0;
}

/*
 * Capture the unhandled instruction in two places:
 *
 *  - `first_unhandled_*[hartid]` — the very first trap this boot.
 *    Sticky once set so cascading SIGILLs don't overwrite the
 *    root cause.
 *
 *  - `unhandled_table[hartid][...]` — a per-hart dedup table of
 *    *all* unique encodings the emulator couldn't handle this
 *    boot. Dedup is by `insn` only (not by PC) so a single
 *    encoding hit from many call sites consumes one entry — the
 *    operator's question "what kinds of insns are missing" gets
 *    a clean N-row histogram even on the busiest workload.
 *
 * The table is small enough (32 entries × 1 hart in practice) that
 * a linear scan per trap is cheap relative to the M-mode trap
 * round-trip we're already paying. When the table fills up, new
 * encodings increment unhandled_overflow[hartid] instead of
 * evicting existing entries: the first N kinds we saw are sticky,
 * which is what the operator's debug workflow wants.
 */
void sbi_insn_emu_pmu_capture_unhandled(ulong insn, ulong mepc)
{
	u32 hartid = current_hartid();
	struct bhx_emu_pmu_unhandled_entry *table;
	int i, empty_slot = -1;

	if (hartid >= BHX_EMU_PMU_MAX_HARTS)
		return;

	if (bhx_emu_pmu_publish.first_unhandled_insn[hartid] == 0) {
		bhx_emu_pmu_publish.first_unhandled_insn[hartid] = insn;
		bhx_emu_pmu_publish.first_unhandled_mepc[hartid] = mepc;
	}

	table = bhx_emu_pmu_publish.unhandled_table[hartid];
	for (i = 0; i < BHX_EMU_PMU_UNHANDLED_TABLE_SIZE; i++) {
		if (table[i].insn == insn) {
			table[i].count++;
			return;
		}
		if (table[i].insn == 0 && empty_slot < 0)
			empty_slot = i;
	}
	if (empty_slot >= 0) {
		table[empty_slot].insn = insn;
		table[empty_slot].first_mepc = mepc;
		table[empty_slot].count = 1;
	} else {
		bhx_emu_pmu_publish.unhandled_overflow[hartid]++;
	}
}
