// Export decompiled C for every function, plus a function index, to the directory given as script arg.
//@category AT32

import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.FunctionIterator;
import java.io.File;
import java.io.PrintWriter;

public class ExportDecomp extends GhidraScript {
    @Override
    protected void run() throws Exception {
        String[] args = getScriptArgs();
        File outDir = new File(args.length > 0 ? args[0] : ".");
        outDir.mkdirs();
        DecompInterface di = new DecompInterface();
        di.openProgram(currentProgram);
        try (PrintWriter c = new PrintWriter(new File(outDir, "decomp.c"));
             PrintWriter idx = new PrintWriter(new File(outDir, "functions.tsv"))) {
            idx.println("address\tsize\tname\tcallers\tcallees");
            FunctionIterator it = currentProgram.getFunctionManager().getFunctions(true);
            int n = 0;
            while (it.hasNext() && !monitor.isCancelled()) {
                Function f = it.next();
                idx.printf("%s\t%d\t%s\t%d\t%d%n", f.getEntryPoint(), f.getBody().getNumAddresses(),
                    f.getName(), f.getCallingFunctions(monitor).size(), f.getCalledFunctions(monitor).size());
                DecompileResults r = di.decompileFunction(f, 60, monitor);
                c.println("// ==== " + f.getName() + " @ " + f.getEntryPoint() + " ====");
                if (r != null && r.decompileCompleted()) {
                    c.println(r.getDecompiledFunction().getC());
                } else {
                    c.println("// decompile failed: " + (r == null ? "null" : r.getErrorMessage()));
                }
                n++;
            }
            println("Exported " + n + " functions to " + outDir);
        } finally {
            di.dispose();
        }
    }
}
