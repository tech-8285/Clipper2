#include <gtest/gtest.h>
#include <cmath>
#include <set>
#include "clipper2/clipper.h"
#include "clipper2/clipper.triangulation.h"

using namespace Clipper2Lib;

namespace {

using PointSet = std::set<Point64,
  bool (*)(const Point64&, const Point64&)>;

PointSet MakePointSet() {
  return PointSet([](const Point64& a, const Point64& b) {
    return a.x != b.x ? a.x < b.x : a.y < b.y;
  });
}

PointSet OutlineVertices(const Paths64& pp) {
  PointSet s = MakePointSet();
  for (const Path64& path : pp) {
    for (size_t i = 0; i < path.size(); ++i) {
      size_t prev = (i + path.size() - 1) % path.size();
      if (path[i] == path[prev]) continue; // consecutive duplicates
      s.insert(path[i]);
    }
  }
  return s;
}

PointSet MeshVertices(const Paths64& tris) {
  PointSet s = MakePointSet();
  for (const Path64& t : tris)
    for (const Point64& p : t) s.insert(p);
  return s;
}

bool AllPositiveArea(const Paths64& tris) {
  for (const Path64& t : tris)
    if (CrossProductSign(t[0], t[1], t[2]) == 0) return false;
  return true;
}

double MeshArea(const Paths64& tris) {
  double a = 0.0;
  for (const Path64& t : tris) a += std::fabs(Area(t));
  return a;
}

struct EdgeKey {
  Point64 a, b;
  bool operator<(const EdgeKey& o) const {
    if (!(a == o.a))
      return a.x != o.a.x ? a.x < o.a.x : a.y < o.a.y;
    return b.x != o.b.x ? b.x < o.b.x : b.y < o.b.y;
  }
};

EdgeKey MakeKey(const Point64& x, const Point64& y) {
  return (x.x < y.x || (x.x == y.x && x.y < y.y))
    ? EdgeKey{x, y} : EdgeKey{y, x};
}

bool OutlineEdgesInMesh(const Paths64& pp, const Paths64& tris) {
  std::set<EdgeKey> meshEdges;
  for (const Path64& t : tris)
    for (int k = 0; k < 3; ++k)
      meshEdges.insert(MakeKey(t[k], t[(k + 1) % 3]));
  for (const Path64& path : pp)
    for (size_t i = 0; i < path.size(); ++i) {
      Point64 a = path[i];
      Point64 b = path[(i + 1) % path.size()];
      if (a == b) continue;
      if (meshEdges.find(MakeKey(a, b)) == meshEdges.end())
        return false;
    }
  return true;
}

} // namespace

TEST(Clipper2Tests, TriangulateDefaultDropsCollinearVertices) {
  // a square with midpoints inserted into every straight side
  Paths64 pp;
  pp.push_back(Path64{
    {0, 0}, {10, 0}, {20, 0},
    {20, 10}, {20, 20},
    {10, 20}, {0, 20},
    {0, 10}});

  Paths64 sol;
  ASSERT_EQ(Triangulate(pp, sol), TriangulateResult::success);
  // default behaviour keeps only the 4 corners
  EXPECT_EQ(MeshVertices(sol).size(), 4u);
  EXPECT_EQ(sol.size(), 2u);
  EXPECT_NEAR(MeshArea(sol), 400.0, 1e-9);
}

TEST(Clipper2Tests, TriangulateIncludeCollinearKeepsOutlineVertices) {
  Paths64 pp;
  pp.push_back(Path64{
    {0, 0}, {10, 0}, {20, 0},
    {20, 10}, {20, 20},
    {10, 20}, {0, 20},
    {0, 10}});

  for (bool delaunay : {false, true}) {
    Paths64 sol;
    ASSERT_EQ(Triangulate(pp, sol, delaunay, true),
      TriangulateResult::success);
    // every distinct outline vertex (8) is preserved
    EXPECT_EQ(MeshVertices(sol), OutlineVertices(pp));
    EXPECT_EQ(MeshVertices(sol).size(), 8u);
    // 8 boundary vertices -> 6 triangles
    EXPECT_EQ(sol.size(), 6u);
    EXPECT_TRUE(AllPositiveArea(sol));
    EXPECT_NEAR(MeshArea(sol), 400.0, 1e-9);
    // the collinear sub-segments survive as constraint edges
    EXPECT_TRUE(OutlineEdgesInMesh(pp, sol));
  }
}

TEST(Clipper2Tests, TriangulateIncludeCollinearWithHole) {
  Paths64 pp;
  // clockwise outer outline with midpoints on its sides
  pp.push_back(Path64{
    {0, 0}, {50, 0}, {100, 0},
    {100, 50}, {100, 100},
    {50, 100}, {0, 100}, {0, 50}});
  // counter-clockwise inner outline (hole)
  pp.push_back(Path64{{25, 25}, {25, 75}, {75, 75}, {75, 25}});

  Paths64 sol;
  ASSERT_EQ(Triangulate(pp, sol, true, true), TriangulateResult::success);
  EXPECT_EQ(MeshVertices(sol), OutlineVertices(pp));
  EXPECT_TRUE(AllPositiveArea(sol));
  EXPECT_TRUE(OutlineEdgesInMesh(pp, sol));
  // outer area 10000 minus hole area 2500
  EXPECT_NEAR(MeshArea(sol), 7500.0, 1e-6);
  // V=12 vertices, H=1 hole  =>  T = V + 2H - 2 = 12
  EXPECT_EQ(sol.size(), 12u);
}

TEST(Clipper2Tests, TriangulateIncludeCollinearDoubleInterface) {
  PathsD pp;
  pp.push_back(PathD{
    PointD(0.0, 0.0), PointD(0.5, 0.0), PointD(1.0, 0.0),
    PointD(1.0, 0.5), PointD(1.0, 1.0),
    PointD(0.5, 1.0), PointD(0.0, 1.0),
    PointD(0.0, 0.5)});

  PathsD solKeep;
  ASSERT_EQ(Triangulate(pp, 6, solKeep, true, true),
    TriangulateResult::success);
  bool hasMidpoint = false;
  for (const PathD& t : solKeep)
    for (const PointD& p : t)
      if (std::fabs(p.x - 0.5) < 1e-9 && std::fabs(p.y) < 1e-9)
        hasMidpoint = true;
  EXPECT_TRUE(hasMidpoint);
  double area = 0.0;
  for (const PathD& t : solKeep) area += std::fabs(Area(t));
  EXPECT_NEAR(area, 1.0, 1e-9);

  // default mode must still remove the midpoint
  PathsD solDefault;
  ASSERT_EQ(Triangulate(pp, 6, solDefault), TriangulateResult::success);
  bool midpointGone = true;
  for (const PathD& t : solDefault)
    for (const PointD& p : t)
      if (std::fabs(p.x - 0.5) < 1e-9 && std::fabs(p.y) < 1e-9)
        midpointGone = false;
  EXPECT_TRUE(midpointGone);
}

TEST(Clipper2Tests, TriangulateIncludeCollinearSharedBoundary) {
  // two squares that share the vertical edge x=10 (with a midpoint)
  Paths64 leftPaths, rightPaths;
  leftPaths.push_back(Path64{{0, 0}, {10, 0}, {10, 10}, {10, 20}, {0, 20}});
  rightPaths.push_back(Path64{{10, 0}, {20, 0}, {20, 20}, {10, 20}, {10, 10}});

  Paths64 leftSol, rightSol;
  ASSERT_EQ(Triangulate(leftPaths, leftSol, true, true),
    TriangulateResult::success);
  ASSERT_EQ(Triangulate(rightPaths, rightSol, true, true),
    TriangulateResult::success);

  PointSet vl = MeshVertices(leftSol);
  PointSet vr = MeshVertices(rightSol);
  for (const Point64& shared : {Point64(10, 0), Point64(10, 10), Point64(10, 20)}) {
    EXPECT_NE(vl.find(shared), vl.end());
    EXPECT_NE(vr.find(shared), vr.end());
  }
  EXPECT_TRUE(OutlineEdgesInMesh(leftPaths, leftSol));
  EXPECT_TRUE(OutlineEdgesInMesh(rightPaths, rightSol));
}

TEST(Clipper2Tests, TriangulateIncludeCollinearHandlesDuplicateVertices) {
  Paths64 pp;
  pp.push_back(Path64{{0, 0}, {10, 0}, {10, 0}, {20, 0}, {20, 20}, {0, 20}});
  Paths64 sol;
  ASSERT_EQ(Triangulate(pp, sol, true, true), TriangulateResult::success);
  EXPECT_TRUE(AllPositiveArea(sol));
  EXPECT_NEAR(MeshArea(sol), 400.0, 1e-9);
  EXPECT_EQ(MeshVertices(sol), OutlineVertices(pp));
}
