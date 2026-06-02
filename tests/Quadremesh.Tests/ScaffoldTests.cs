using Quadremesh.Core;
using Xunit;

namespace Quadremesh.Tests;

/// <summary>
/// Placeholder tests proving the build graph compiles and the test runner executes (P0 acceptance).
/// Real unit + golden-file tests arrive from P1 onward.
/// </summary>
public class ScaffoldTests
{
    [Fact]
    public void Core_exposes_a_non_empty_version()
    {
        Assert.False(string.IsNullOrWhiteSpace(BuildInfo.Version));
    }
}
