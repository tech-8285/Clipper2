using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Clipper2Lib.UnitTests
{

  [TestClass]
  public class TestTriangulation
  {
    private static HashSet<Point64> MeshVertices(Paths64 tris)
    {
      HashSet<Point64> s = new HashSet<Point64>();
      foreach (Path64 t in tris)
        foreach (Point64 p in t) s.Add(p);
      return s;
    }

    private static HashSet<Point64> OutlineVertices(Paths64 pp)
    {
      HashSet<Point64> s = new HashSet<Point64>();
      foreach (Path64 path in pp)
        for (int i = 0; i < path.Count; ++i)
        {
          int prev = (i + path.Count - 1) % path.Count;
          if (path[i].Equals(path[prev])) continue; // consecutive duplicates
          s.Add(path[i]);
        }
      return s;
    }

    private static bool AllPositiveArea(Paths64 tris)
    {
      foreach (Path64 t in tris)
        if (InternalClipper.CrossProductSign(t[0], t[1], t[2]) == 0)
          return false;
      return true;
    }

    private static double MeshArea(Paths64 tris)
    {
      double a = 0.0;
      foreach (Path64 t in tris) a += Math.Abs(Clipper.Area(t));
      return a;
    }

    private static bool OutlineEdgesInMesh(Paths64 pp, Paths64 tris)
    {
      HashSet<(long, long, long, long)> meshEdges = new();
      foreach (Path64 t in tris)
        for (int k = 0; k < 3; ++k)
        {
          Point64 x = t[k], y = t[(k + 1) % 3];
          if (x.X < y.X || (x.X == y.X && x.Y < y.Y))
            meshEdges.Add((x.X, x.Y, y.X, y.Y));
          else
            meshEdges.Add((y.X, y.Y, x.X, x.Y));
        }
      foreach (Path64 path in pp)
        for (int i = 0; i < path.Count; ++i)
        {
          Point64 a = path[i];
          Point64 b = path[(i + 1) % path.Count];
          if (a.Equals(b)) continue;
          var key = (Math.Min(a.X, b.X), Math.Min(a.Y, b.Y),
                     Math.Max(a.X, b.X), Math.Max(a.Y, b.Y));
          if (!meshEdges.Contains(key)) return false;
        }
      return true;
    }

    [TestMethod]
    public void TestDefaultDropsCollinearVertices()
    {
      Paths64 pp = new();
      pp.Add(new Path64
      {
        new(0, 0), new(10, 0), new(20, 0),
        new(20, 10), new(20, 20),
        new(10, 20), new(0, 20),
        new(0, 10)
      });

      Assert.AreEqual(TriangulateResult.success, Clipper.Triangulate(pp, out Paths64 sol));
      Assert.AreEqual(4, MeshVertices(sol).Count);
      Assert.AreEqual(2, sol.Count);
      Assert.AreEqual(400.0, MeshArea(sol), 1e-9);
    }

    [TestMethod]
    public void TestIncludeCollinearKeepsOutlineVertices()
    {
      Paths64 pp = new();
      pp.Add(new Path64
      {
        new(0, 0), new(10, 0), new(20, 0),
        new(20, 10), new(20, 20),
        new(10, 20), new(0, 20),
        new(0, 10)
      });

      foreach (bool delaunay in new[] { false, true })
      {
        Assert.AreEqual(TriangulateResult.success,
          Clipper.Triangulate(pp, out Paths64 sol, delaunay, true));
        Assert.IsTrue(MeshVertices(sol).SetEquals(OutlineVertices(pp)));
        Assert.AreEqual(8, MeshVertices(sol).Count);
        Assert.AreEqual(6, sol.Count);
        Assert.IsTrue(AllPositiveArea(sol));
        Assert.AreEqual(400.0, MeshArea(sol), 1e-9);
        Assert.IsTrue(OutlineEdgesInMesh(pp, sol));
      }
    }

    [TestMethod]
    public void TestIncludeCollinearWithHole()
    {
      Paths64 pp = new();
      pp.Add(new Path64
      {
        new(0, 0), new(50, 0), new(100, 0),
        new(100, 50), new(100, 100),
        new(50, 100), new(0, 100), new(0, 50)
      });
      pp.Add(new Path64 { new(25, 25), new(25, 75), new(75, 75), new(75, 25) });

      Assert.AreEqual(TriangulateResult.success,
        Clipper.Triangulate(pp, out Paths64 sol, true, true));
      Assert.IsTrue(MeshVertices(sol).SetEquals(OutlineVertices(pp)));
      Assert.IsTrue(AllPositiveArea(sol));
      Assert.IsTrue(OutlineEdgesInMesh(pp, sol));
      Assert.AreEqual(7500.0, MeshArea(sol), 1e-6);
      Assert.AreEqual(12, sol.Count);
    }

    [TestMethod]
    public void TestIncludeCollinearSharedBoundary()
    {
      Paths64 leftPaths = new();
      leftPaths.Add(new Path64 { new(0, 0), new(10, 0), new(10, 10), new(10, 20), new(0, 20) });
      Paths64 rightPaths = new();
      rightPaths.Add(new Path64 { new(10, 0), new(20, 0), new(20, 20), new(10, 20), new(10, 10) });

      Assert.AreEqual(TriangulateResult.success,
        Clipper.Triangulate(leftPaths, out Paths64 leftSol, true, true));
      Assert.AreEqual(TriangulateResult.success,
        Clipper.Triangulate(rightPaths, out Paths64 rightSol, true, true));

      HashSet<Point64> vl = MeshVertices(leftSol);
      HashSet<Point64> vr = MeshVertices(rightSol);
      foreach (Point64 shared in new[] { new Point64(10, 0), new Point64(10, 10), new Point64(10, 20) })
      {
        Assert.IsTrue(vl.Contains(shared));
        Assert.IsTrue(vr.Contains(shared));
      }
      Assert.IsTrue(OutlineEdgesInMesh(leftPaths, leftSol));
      Assert.IsTrue(OutlineEdgesInMesh(rightPaths, rightSol));
    }

    [TestMethod]
    public void TestIncludeCollinearHandlesDuplicateVertices()
    {
      Paths64 pp = new();
      pp.Add(new Path64 { new(0, 0), new(10, 0), new(10, 0), new(20, 0), new(20, 20), new(0, 20) });
      Assert.AreEqual(TriangulateResult.success,
        Clipper.Triangulate(pp, out Paths64 sol, true, true));
      Assert.IsTrue(AllPositiveArea(sol));
      Assert.AreEqual(400.0, MeshArea(sol), 1e-9);
      Assert.IsTrue(MeshVertices(sol).SetEquals(OutlineVertices(pp)));
    }
  }
}
