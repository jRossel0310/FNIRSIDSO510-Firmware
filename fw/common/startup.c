#include <stdint.h>

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
int main(void);

void Reset_Handler(void) {
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0;
    main();
    for (;;) { }
}

void Default_Handler(void) { for (;;) { } }

// Interrupts stay disabled (cpsid i); every vector points at Default_Handler.
__attribute__((section(".isr_vector"), used))
const void *const vectors[16 + 84] = {
    [0] = &_estack,
    [1] = Reset_Handler,
    [2 ... 16 + 84 - 1] = Default_Handler,
};

// Minimal libc pieces GCC may emit calls to (we link with -nostdlib).
#include <stddef.h>
void *memset(void *d, int c, size_t n) { unsigned char *p = d; while (n--) *p++ = (unsigned char)c; return d; }
void *memcpy(void *d, const void *s, size_t n) { unsigned char *p = d; const unsigned char *q = s; while (n--) *p++ = *q++; return d; }
