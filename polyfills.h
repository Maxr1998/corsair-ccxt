#pragma once

#include <stddef.h>

/* TYPES */
#define __KERNEL_DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#define BITS_PER_BYTE		8
#define BITS_PER_TYPE(type)	(sizeof(type) * BITS_PER_BYTE)
#define BITS_TO_LONGS(nr)	__KERNEL_DIV_ROUND_UP(nr, BITS_PER_TYPE(long))
#define DECLARE_BITMAP(name,bits) \
unsigned long name[BITS_TO_LONGS(bits)]

/* BITOPS */
static void set_bit(const long nr, volatile unsigned long *addr)
{
	const unsigned long mask = 1UL << (nr % BITS_PER_TYPE(long));
	volatile unsigned long *p = addr + nr / BITS_PER_TYPE(long);

	*p |= mask;
}

/* STDIO */
static int scnprintf(char *buf, size_t size, const char *fmt, ...)
{
	return -1;
}

/* HID IO */
#define hid_notice(dev, fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)

/* MUTEX */
#define mutex_lock(mutex)
#define mutex_unlock(mutex)

/* MATH */
#define min(a, b) ((a) < (b) ? (a) : (b))

/*
 * Divide positive or negative dividend by positive or negative divisor
 * and round to closest integer. Result is undefined for negative
 * divisors if the dividend variable type is unsigned and for negative
 * dividends if the divisor variable type is unsigned.
 */
#define DIV_ROUND_CLOSEST(x, divisor)                                \
	({                                                           \
		typeof(x) __x = x;                                   \
		typeof(divisor) __d = divisor;                       \
		(((typeof(x))-1) > 0 || ((typeof(divisor))-1) > 0 || \
		 (((__x) > 0) == ((__d) > 0))) ?                     \
			(((__x) + ((__d) / 2)) / (__d)) :            \
			(((__x) - ((__d) / 2)) / (__d));             \
	})