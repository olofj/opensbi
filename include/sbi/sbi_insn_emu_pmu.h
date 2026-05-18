/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Per-extension hit counters for the trap-based ISA emulator
 * (sbi_insn_emu*.c). Generic interface: each emulator entry point
 * tracks which extension matched and calls `sbi_insn_emu_pmu_inc()`
 * on the way out. The implementation is supplied by a platform-
 * specific PMU device (e.g. `bhx_emu_pmu.c`) that decides how to
 * expose the counts — typically via OpenSBI's platform-firmware-event
 * PMU mechanism (event_code = SBI_PMU_FW_PLATFORM).
 *
 * When no platform implementation registers, the default is a no-op
 * stub so the emulator's instrumentation costs zero.
 */

#ifndef __SBI_INSN_EMU_PMU_H__
#define __SBI_INSN_EMU_PMU_H__

#include <sbi/sbi_types.h>

/*
 * Extension identifiers passed to sbi_insn_emu_pmu_inc(). One per
 * emulated extension, plus NONE (sentinel for un-classified call
 * sites), ANY (aggregate), and UNHANDLED (illegal-insn traps the
 * emulator could not handle). Append-only — never re-purpose an
 * existing value; downstream PMU devices use these as wire-format
 * sub-event IDs.
 */
enum sbi_insn_emu_ext {
	SBI_INSN_EMU_EXT_NONE		= 0,
	SBI_INSN_EMU_EXT_ANY		= 1,
	SBI_INSN_EMU_EXT_ZCB		= 2,
	SBI_INSN_EMU_EXT_ZBA		= 3,
	SBI_INSN_EMU_EXT_ZBB		= 4,
	SBI_INSN_EMU_EXT_ZBS		= 5,
	SBI_INSN_EMU_EXT_ZICOND		= 6,
	SBI_INSN_EMU_EXT_ZIMOP		= 7,
	SBI_INSN_EMU_EXT_ZAWRS		= 8,
	SBI_INSN_EMU_EXT_ZFA		= 9,
	SBI_INSN_EMU_EXT_ZFHMIN		= 10,
	SBI_INSN_EMU_EXT_ZICBOM		= 11,
	SBI_INSN_EMU_EXT_ZICBOZ		= 12,
	SBI_INSN_EMU_EXT_ZCMOP		= 13,
	SBI_INSN_EMU_EXT_ZBC		= 14,
	SBI_INSN_EMU_EXT_SUPM		= 15,
	/*
	 * Counts illegal-instruction traps that reached
	 * truly_illegal_insn() — i.e. the emulator either had no
	 * decode for the insn (e.g. an RVV vandn.vv Zvbb insn lands
	 * on a truly_illegal_insn slot in the dispatcher table) or
	 * dispatched and fell through without a matching case. The
	 * usable invariant is `ANY - sum(handled_ext) == UNHANDLED`.
	 * Userspace perf can sample this counter directly to see how
	 * often the X280 is hitting something the emulator can't
	 * help with — typically a signal that another patch (Zvbb,
	 * Zvbc, …) is needed.
	 */
	SBI_INSN_EMU_EXT_UNHANDLED	= 16,
	/* Vector Basic Bit-manipulation — Freisen v3 3/3. Gating
	 * extension for stock RVA23U64 userspaces (Ubuntu 26.04 /
	 * glibc-RVA23) on the X280, which has RVV 1.0 but no Zvbb. */
	SBI_INSN_EMU_EXT_ZVBB		= 17,
	SBI_INSN_EMU_EXT_MAX
};

/*
 * Increment the counter for `ext` and the aggregate `ANY` counter on
 * the current hart. `ext == SBI_INSN_EMU_EXT_NONE` is a deliberate
 * no-op so call sites can pass an un-classified match-tracker
 * variable without branching. Safe to call from M-mode trap context.
 *
 * Default (weak) implementation is a no-op. Platforms that want to
 * expose counts via SBI PMU override by providing a strong
 * definition.
 */
void sbi_insn_emu_pmu_inc(int ext);

/*
 * Capture-hook for illegal-insn traps that reached
 * truly_illegal_insn() without being emulated. The default weak
 * implementation is a no-op; a platform-specific PMU device can
 * override to record the insn encoding + mepc for the operator to
 * disassemble. Called once per such trap, alongside the UNHANDLED
 * counter bump. `insn` is the trapped instruction encoding, `mepc`
 * is the guest PC it lives at.
 */
void sbi_insn_emu_pmu_capture_unhandled(ulong insn, ulong mepc);

/*
 * Cold + warm boot hook. Called unconditionally from sbi_init.c
 * after sbi_pmu_init() on both the cold-boot and warm-boot paths.
 * The default weak implementation is a no-op; a platform-specific
 * PMU device file overrides this with a strong definition that
 * registers an sbi_pmu_device. Idempotent across harts.
 */
int sbi_insn_emu_pmu_init(void);

#endif /* __SBI_INSN_EMU_PMU_H__ */
