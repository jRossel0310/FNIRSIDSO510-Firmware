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
