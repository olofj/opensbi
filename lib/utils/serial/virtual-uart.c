/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Tenstorrent Inc.
 *
 * Authors:
 *   Joel Smith <joelsmith@tenstorrent.com>
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_io.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_debug_descriptor.h>

static const u64 VIRTUAL_UART_MAGIC = 0x5649525455415254ULL;

#define BUFFER_SIZE 0x1000
struct __attribute__((packed, aligned(4))) virtual_uart_queues {
	volatile le64_t magic;
	volatile char tx_buf[BUFFER_SIZE];
	volatile char rx_buf[BUFFER_SIZE];
	volatile le32_t tx_head;
	volatile le32_t tx_tail;
	volatile le32_t rx_head;
	volatile le32_t rx_tail;
};

static struct virtual_uart_queues virtual_uart;

static inline u32 read_ptr(volatile u32 *ptr)
{
	u32 value = *ptr;
	smp_rmb();
	return value;
}

static inline void write_ptr(volatile u32 *ptr, u32 value)
{
	smp_wmb();
	*ptr = value;
}

static void virtual_uart_putc(char ch)
{
	volatile struct virtual_uart_queues *uart = &virtual_uart;
	u32 head = read_ptr(&uart->tx_head);
	u32 tail = read_ptr(&uart->tx_tail);
	u32 next_head = (head + 1) % BUFFER_SIZE;

	/* wait for the buffer to have space */
	while (next_head == tail) {
		tail = read_ptr(&uart->tx_tail);
	}

	if (next_head != tail) {
		uart->tx_buf[uart->tx_head] = ch;
		write_ptr(&uart->tx_head, next_head);
	}
}

static int virtual_uart_getc(void)
{
	volatile struct virtual_uart_queues *uart = &virtual_uart;
	u32 head = read_ptr(&uart->rx_head);
	u32 tail = read_ptr(&uart->rx_tail);

	if (head != tail) {
		int ch = uart->rx_buf[tail];
		smp_mb();
		write_ptr(&uart->rx_tail, (tail + 1) % BUFFER_SIZE);
		return ch;
	}

	return -1;
}

static struct sbi_console_device virtual_uart_console = {
	.name = "virtual_uart",
	.console_putc = virtual_uart_putc,
	.console_getc = virtual_uart_getc
};

int virtual_uart_init()
{
	volatile struct virtual_uart_queues *uart = &virtual_uart;

	uart->magic = VIRTUAL_UART_MAGIC;
	uart->tx_head = 0;
	uart->tx_tail = 0;
	uart->rx_head = 0;
	uart->rx_tail = 0;

	debug_descriptor.virtuart_base = (u64)&virtual_uart;
	sbi_console_set_device(&virtual_uart_console);

	return 0;
}
