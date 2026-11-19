/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Date      :  6 December 2025                                                 *
* Release   :  BETA RELEASE                                                    *
* Website   :  https://www.angusj.com                                          *
* Copyright :  Angus Johnson 2010-2025                                         *
* Purpose   :  Delaunay Triangulation                                          *
* License   :  https://www.boost.org/LICENSE_1_0.txt                           *
*******************************************************************************/

#ifndef CLIPPER_TRIANGULATION_H
#define CLIPPER_TRIANGULATION_H

#include <stack>
#include "clipper2/clipper.core.h"

namespace Clipper2Lib
{

  enum class TriangulateResult { success, fail, no_polygons, paths_intersect };

  // Triangulate - this function will not accept intesecting paths
  //
  // When 'includeCollinear' is false (the default), collinear vertices that
  // lie in the middle of a straight outline segment are removed before
  // triangulation (the original behaviour).
  //
  // When 'includeCollinear' is true, every distinct vertex in the supplied
  // polygon outlines (including vertices that sit on a straight outline
  // segment) is preserved and becomes a vertex of the resulting triangles.
  // The outline edges that connect these vertices are retained as fixed
  // constraint edges, so adjacent meshes that share an outline keep matching
  // vertices and join without cracks.
  TriangulateResult Triangulate(const Paths64& pp, Paths64& solution,
    bool useDelaunay = true, bool includeCollinear = false);
  TriangulateResult Triangulate(const PathsD& pp, int decPlaces, PathsD& solution,
    bool useDelaunay = true, bool includeCollinear = false);

} // Clipper2Lib namespace
#endif  // CLIPPER_TRIANGULATION_H
