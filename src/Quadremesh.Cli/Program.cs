using System.Globalization;
using Quadremesh.Core;
using Quadremesh.Core.Mesh;
using Quadremesh.IO;

// Minimal CLI for the P5 demoable milestone: triangle OBJ in, quad-dominant OBJ out.
// The full option surface (adaptive sizing, features, guides, symmetry, progress) arrives in P9.
if (args.Length < 2)
{
    Console.Error.WriteLine("usage: quadremesh <in.obj> <out.obj> [--target <quads> | --edge <length>]");
    Console.Error.WriteLine($"  quadremesh {BuildInfo.Version}");
    return 1;
}

string inputPath = args[0];
string outputPath = args[1];
var options = new QuadRemesherOptions { TargetQuadCount = 5000 };

for (int i = 2; i < args.Length - 1; i++)
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
        default:
            Console.Error.WriteLine($"unknown option: {args[i]}");
            return 1;
    }
}

TriangleMesh input = ObjReader.ReadTriangles(inputPath);
Console.WriteLine($"loaded {input.VertexCount} verts, {input.TriangleCount} tris from {inputPath}");

QuadMesh output = QuadRemesher.Remesh(input, options);
ObjWriter.Write(outputPath, output);

Console.WriteLine($"wrote {output.VertexCount} verts, {output.FaceCount} faces " +
    $"({output.QuadFraction:P0} quads) to {outputPath}");
return 0;
