# Ghidra headless analysis

```sh
~/tools/ghidra_extract/ghidra_12.2_PUBLIC/support/analyzeHeadless ghidra_project dso510 \
  -import APP_DSO-510_V2.0.5_250114.bin -overwrite \
  -loader BinaryLoader -loader-baseAddr 0x08007000 -processor ARM:LE:32:Cortex \
  -scriptPath tools/ghidra -postScript SetupAT32.java -postScript ExportDecomp.java re
```

- `SetupAT32.java`: marks flash read-only (so literal-pool constants fold), adds SRAM/peripheral
  blocks + AT32F403A peripheral labels, creates functions from the vector table, runs analysis.
- `ExportDecomp.java <outdir>`: writes `decomp.c` (all functions) and `functions.tsv`.

GUI: `~/tools/ghidra_extract/ghidra_12.2_PUBLIC/ghidraRun`, open `ghidra_project/dso510.gpr`.
