using Quadremesh.CorpusGen;
using Quadremesh.Core.Mesh;
using Quadremesh.IO;

// Writes the procedural corpus to the given directory (default: tests/corpus relative to cwd).
// Deterministic: re-running produces byte-identical files. Regenerate with:
//   dotnet run --project tools/corpus-gen
string outputDir = args.Length > 0 ? args[0] : Path.Combine("tests", "corpus");
Directory.CreateDirectory(outputDir);

foreach ((string name, TriangleMesh mesh) in Primitives.All())
{
    string path = Path.Combine(outputDir, name + ".obj");
    ObjWriter.Write(path, mesh);
    Console.WriteLine($"{name,-14} -> {path}  ({mesh.VertexCount} verts, {mesh.TriangleCount} tris)");
}

Console.WriteLine("Corpus written.");
