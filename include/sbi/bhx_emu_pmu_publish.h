/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Host-readable mirror of the bhx ISA-emulation PMU counters.
 *
 * The bhx PMU device's read-side path (sbi_pmu_device fw_counter_*)
 * is the SBI-ecall route used by `perf` inside the guest. This
 * struct adds a *second* read route for the bhx host: an aligned,
 * statically-placed counter array that the host can scrape over
 * PCIe without any guest cooperation — useful when the guest is
 * wedged, hasn't booted to a userspace where perf would run, or
 * the operator just wants a one-shot "what's the emu firing on
 * right now" snapshot from outside.
 *
 * Both read paths see the same memory: sbi_insn_emu_pmu_inc()
 * updates this struct in place. The host locates it by reading
 * the 8-byte pointer that fw_base.S parks at fw_jump.bin + 0x88
 * (sibling slot to the debug_descriptor pointer at +0x80).
 *
 * Wire-format invariants — never reorder, only append:
 *   magic       = BHX_EMU_PMU_MAGIC ("BPMU" little-endian)
 *   version     = 1 today; bump for any layout change
 *   max_harts   = static upper bound on hartid (X280 has 1/L2CPU)
 *   max_events  = SBI_INSN_EMU_EXT_MAX at firmware build time
 *   counts[h][e] = per-hart, per-event 64-bit counter
 */

#ifndef __BHX_EMU_PMU_PUBLISH_H__
#define __BHX_EMU_PMU_PUBLISH_H__

#include <sbi/sbi_ecall_interface.h>
#include <sbi/sbi_insn_emu_pmu.h>
#include <sbi/sbi_pmu.h>
#include <sbi/sbi_types.h>

/* "BPMU" stored little-endian. The host validates this byte-for-byte
 * before trusting the rest of the struct. */
#define BHX_EMU_PMU_MAGIC	0x554d5042u
#define BHX_EMU_PMU_VERSION	3u

/* Upper bound on hartid we track. X280 has 1 hart per L2CPU so a
 * given firmware image only ever sees hartid == 0; sized generously
 * so a future multi-hart platform doesn't truncate. */
#define BHX_EMU_PMU_MAX_HARTS	8

struct bhx_emu_pmu_publish {
	u32	magic;		/* BHX_EMU_PMU_MAGIC */
	u32	version;	/* BHX_EMU_PMU_VERSION */
	u32	max_harts;	/* BHX_EMU_PMU_MAX_HARTS */
	u32	max_events;	/* SBI_INSN_EMU_EXT_MAX */

	/* Per-hart counter array. counts[h][SBI_INSN_EMU_EXT_ANY]
	 * tracks every entry into the emulator path; per-extension
	 * slots track successful emulation; UNHANDLED tracks
	 * truly_illegal_insn() hits. */
	u64	counts[BHX_EMU_PMU_MAX_HARTS][SBI_INSN_EMU_EXT_MAX];

	/* PMU-side counter-binding table — perf invokes counter_start
	 * to bind a hardware-counter slot to a sub-event ID. Mirrors
	 * the old scratch-state field; kept here so the published
	 * struct is self-contained. */
	int	ctr_to_event[BHX_EMU_PMU_MAX_HARTS][SBI_PMU_FW_CTR_MAX];

	/* Per-hart capture of the first illegal-insn trap that
	 * truly_illegal_insn() couldn't handle. Useful for "which
	 * extension's emulator do I need to write next" — the host
	 * disassembles `first_unhandled_insn` and gets a direct
	 * answer instead of grepping guest dmesg for SIGILL bytes.
	 *
	 * Set once per boot (write only if the slot is still zero)
	 * so the first sticky failure isn't overwritten by later
	 * cascading SIGILLs from the same workload.
	 *
	 * `first_unhandled_mepc` is the guest PC the instruction
	 * lives at, useful for cross-referencing against
	 * /proc/.../maps + objdump output inside the guest.
	 *
	 * Both zeroed by sbi_insn_emu_pmu_init() on every hart entry
	 * (cold + warm reboot), so they reflect "this boot" rather
	 * than "ever since this fw_jump.bin loaded." */
	u64	first_unhandled_insn[BHX_EMU_PMU_MAX_HARTS];
	u64	first_unhandled_mepc[BHX_EMU_PMU_MAX_HARTS];
};

extern struct bhx_emu_pmu_publish bhx_emu_pmu_publish;

#endif /* __BHX_EMU_PMU_PUBLISH_H__ */
