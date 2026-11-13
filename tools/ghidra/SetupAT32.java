// Ghidra headless post-script: set up an AT32 (Cortex-M4) raw app image and analyze it.
// Assumes the image was imported as raw binary at its load address (e.g. 0x08007000).
//@category AT32

import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.mem.Memory;
import ghidra.program.model.mem.MemoryBlock;
import ghidra.program.model.symbol.SourceType;
import ghidra.program.model.data.PointerDataType;

public class SetupAT32 extends GhidraScript {

    private void block(String name, long start, long size, boolean volatil) throws Exception {
        Memory mem = currentProgram.getMemory();
        Address a = toAddr(start);
        if (mem.getBlock(a) != null) return;
        MemoryBlock b = mem.createUninitializedBlock(name, a, size, false);
        b.setRead(true);
        b.setWrite(true);
        b.setExecute(false);
        b.setVolatile(volatil);
    }

    private static final String[] CORE_VECTORS = {
        "_estack", "Reset_Handler", "NMI_Handler", "HardFault_Handler", "MemManage_Handler",
        "BusFault_Handler", "UsageFault_Handler", null, null, null, null, "SVC_Handler",
        "DebugMon_Handler", null, "PendSV_Handler", "SysTick_Handler"
    };

    @Override
    protected void run() throws Exception {
        Address base = currentProgram.getMinAddress();
        println("Image base: " + base);
        // Flash is read-only: lets the decompiler fold literal-pool constants (peripheral addresses).
        MemoryBlock flash = currentProgram.getMemory().getBlock(base);
        flash.setName("FLASH");
        flash.setWrite(false);
        flash.setExecute(true);

        block("SRAM", 0x20000000L, 0x00080000L, false);       // generous: covers F403A/F435 variants
        block("PERIPH", 0x40000000L, 0x00100000L, true);      // APB1/APB2/AHB1
        block("PERIPH_AHB2", 0x50000000L, 0x00100000L, true); // USB OTG on F435
        block("XMC", 0x60000000L, 0x10000000L, true);         // external memory ctl (LCD on 8080 bus?)
        block("XMC_REG", 0xA0000000L, 0x00001000L, true);
        block("SCS", 0xE0000000L, 0x00100000L, true);         // Cortex-M system control space

        String[][] periph = {
            {"TMR2","40000000"},{"TMR3","40000400"},{"TMR4","40000800"},{"TMR5","40000C00"},
            {"TMR6","40001000"},{"TMR7","40001400"},{"TMR12","40001800"},{"TMR13","40001C00"},
            {"TMR14","40002000"},{"ERTC","40002800"},{"WWDT","40002C00"},{"WDT","40003000"},
            {"SPI2","40003800"},{"SPI3","40003C00"},{"USART2","40004400"},{"USART3","40004800"},
            {"UART4","40004C00"},{"UART5","40005000"},{"I2C1","40005400"},{"I2C2","40005800"},
            {"USBD","40005C00"},{"USBD_SRAM","40006000"},{"CAN1","40006400"},{"CAN2","40006800"},
            {"BPR","40006C00"},{"PWC","40007000"},{"DAC","40007400"},{"UART7","40007800"},
            {"UART8","40007C00"},{"IOMUX","40010000"},{"EXINT","40010400"},{"GPIOA","40010800"},
            {"GPIOB","40010C00"},{"GPIOC","40011000"},{"GPIOD","40011400"},{"GPIOE","40011800"},
            {"ADC1","40012400"},{"ADC2","40012800"},{"TMR1","40012C00"},{"SPI1","40013000"},
            {"TMR8","40013400"},{"USART1","40013800"},{"ADC3","40013C00"},{"TMR9","40014C00"},
            {"TMR10","40015000"},{"TMR11","40015400"},{"SDIO1","40018000"},{"DMA1","40020000"},
            {"DMA2","40020400"},{"CRM","40021000"},{"FLASH_REG","40022000"},{"CRC","40023000"},
            {"SDIO2","40023400"},{"XMC_BANK1_REG","A0000000"},
            {"SysTick","E000E010"},{"NVIC","E000E100"},{"SCB","E000ED00"},
        };
        for (String[] p : periph) {
            createLabel(toAddr(Long.parseLong(p[1], 16)), p[0], true, SourceType.USER_DEFINED);
        }

        // Vector table: 16 core + up to 112 IRQs; stop at first entry that isn't a flash pointer/zero.
        long imgEnd = currentProgram.getMaxAddress().getOffset();
        for (int i = 0; i < 128; i++) {
            Address slot = base.add(i * 4L);
            long v = getInt(slot) & 0xffffffffL;
            clearListing(slot, slot.add(3));
            if (i == 0) {
                createData(slot, new PointerDataType());
                createLabel(slot, "vector_table", true, SourceType.USER_DEFINED);
                continue;
            }
            if (v == 0) continue;
            if (v < base.getOffset() || v > imgEnd) {
                println("Vector table ends at index " + i);
                break;
            }
            createData(slot, new PointerDataType());
            Address target = toAddr(v & ~1L);
            String name = (i < 16 && CORE_VECTORS[i] != null) ? CORE_VECTORS[i] : ("IRQ" + (i - 16) + "_Handler");
            if (getFunctionAt(target) == null) {
                disassemble(target);
                createFunction(target, null);
            }
            if (getFunctionAt(target) != null && getFunctionAt(target).getName().startsWith("FUN_")) {
                getFunctionAt(target).setName(name, SourceType.USER_DEFINED);
            } else {
                createLabel(target, name, false, SourceType.USER_DEFINED);
            }
        }

        setAnalysisOption(currentProgram, "ARM Aggressive Instruction Finder", "true");
        println("Running auto-analysis...");
        analyzeAll(currentProgram);
        println("Functions: " + currentProgram.getFunctionManager().getFunctionCount());
    }
}
