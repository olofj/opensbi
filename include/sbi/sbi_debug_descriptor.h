/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Tenstorrent, Inc.
 *
 * Authors:
 *   Joel Smith <joelsmith@tenstorrent.com>
 */

#ifndef __SBI_DEBUG_DESCRIPTOR_H__
#define __SBI_DEBUG_DESCRIPTOR_H__

struct debug_descriptor {
    u8 eye_catcher[8];  /* "OSBIdbug" */
    u32 version;
    u64 virtuart_base;
};

extern struct debug_descriptor debug_descriptor;

#endif
