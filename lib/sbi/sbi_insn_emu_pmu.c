// SPDX-License-Identifier: BSD-2-Clause
/*
 * Weak default implementations of the ISA-emulation PMU hooks. Linked
 * into every build so sbi_insn_emu*.c and sbi_init.c always resolve
 * the symbols; a platform-specific PMU device (e.g. bhx_emu_pmu.c)
 * overrides with strong definitions to count and expose the events.
 *
 * Marked weak rather than `static inline` so callers compile to
 * normal `call` sites that the linker resolves to either these stubs
 * or the platform override — keeping the emu hot path's branch
 * structure identical regardless of which implementation is in use.
 */

#include <sbi/sbi_insn_emu_pmu.h>

__attribute__((weak)) void sbi_insn_emu_pmu_inc(int ext)
{
	(void)ext;
}

__attribute__((weak)) int sbi_insn_emu_pmu_init(void)
{
	return 0;
}
