using Quadremesh.Core;

// P0 scaffold entry point. The full option surface (--target, --adaptive, guides, symmetry, ...)
// and progress reporting arrive in P9; for now this just proves the executable links and runs.
Console.WriteLine($"quadremesh {BuildInfo.Version}");
Console.WriteLine("Scaffold only — the remesh pipeline is not yet implemented (see Agents/Plans).");
return 0;
