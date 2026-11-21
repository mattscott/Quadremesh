using System.Globalization;
using Quadremesh.Core;
using Quadremesh.Core.Mesh;
using Quadremesh.IO;

// Minimal CLI for the P5 demoable milestone: triangle OBJ in, quad-dominant OBJ out.
// The full option surface (adaptive sizing, features, guides, symmetry, progress) arrives in P9.
if (args.Length < 2)
{
    Console.Error.WriteLine(
        "usage: quadremesh <in.obj|.fbx> <out.obj|.fbx> [--target <quads> | --edge <length>] [--pure] [--faithful]");
    Console.Error.WriteLine($"  quadremesh {BuildInfo.Version}");
    return 1;
}

string inputPath = args[0];
string outputPath = args[1];
var options = new QuadRemesherOptions { TargetQuadCount = 5000 };

for (int i = 2; i < args.Length; i++)
{
    switch (args[i])
    {
        case "--target":
            options.TargetQuadCount = int.Parse(args[++i], CultureInfo.InvariantCulture);
            options.TargetEdgeLength = null;
            break;
        case "--edge":
            options.TargetEdgeLength = double.Parse(args[++i], NumberStyles.Float, CultureInfo.InvariantCulture);
            options.TargetQuadCount = null;
            break;
        case "--pure":
            options.Extraction = ExtractionMode.PureQuad;
            break;
        case "--faithful":
            options.FaithfulExtraction = true;
            break;
        default:
            Console.Error.WriteLine($"unknown option: {args[i]}");
            return 1;
    }
}

TriangleMesh input = Path.GetExtension(inputPath).Equals(".fbx", StringComparison.OrdinalIgnoreCase)
    ? FbxReader.ReadTriangles(inputPath)
    : ObjReader.ReadTriangles(inputPath);
Console.WriteLine($"loaded {input.VertexCount} verts, {input.TriangleCount} tris from {inputPath}");

QuadMesh output = QuadRemesher.Remesh(input, options);

if (Path.GetExtension(outputPath).Equals(".fbx", StringComparison.OrdinalIgnoreCase))
{
    FbxWriter.Write(outputPath, output);
}
else
{
    ObjWriter.Write(outputPath, output);
}

Console.WriteLine($"wrote {output.VertexCount} verts, {output.FaceCount} faces " +
    $"({output.QuadFraction:P0} quads) to {outputPath}");
return 0;
