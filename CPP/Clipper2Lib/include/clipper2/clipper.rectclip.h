/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Date      :  5 July 2024                                                     *
* Website   :  https://www.angusj.com                                          *
* Copyright :  Angus Johnson 2010-2024                                         *
* Purpose   :  FAST rectangular clipping                                       *
* License   :  https://www.boost.org/LICENSE_1_0.txt                           *
*******************************************************************************/

#ifndef CLIPPER_RECTCLIP_H
#define CLIPPER_RECTCLIP_H

#include "clipper2/clipper.core.h"
#include <queue>
#ifdef USINGZ
#include <functional>
#endif

namespace Clipper2Lib
{

  // Location: the order is important here, see StartLocsIsClockwise()
  enum class Location { Left, Top, Right, Bottom, Inside };

#ifdef USINGZ
  // This typedef is identical to the ZCallback64 typedef in
  // clipper.engine.h (duplicate identical typedefs are permitted),
  // and it's repeated here so this header remains self-contained.
  typedef std::function<void(
    const Point64& e1bot, const Point64& e1top,
    const Point64& e2bot, const Point64& e2top, Point64& pt)> ZCallback64;
#endif

  class OutPt2;
  typedef std::vector<OutPt2*> OutPt2List;

  class OutPt2 {
  public:
    Point64 pt;
    size_t owner_idx = 0;
    OutPt2List* edge = nullptr;
    OutPt2* next = nullptr;
    OutPt2* prev = nullptr;
  };

  //------------------------------------------------------------------------------
  // RectClip64
  //------------------------------------------------------------------------------

  class RectClip64 {
  private:
    void ExecuteInternal(const Path64& path);
    Path64 GetPath(OutPt2*& op);
  protected:
    const Rect64 rect_;
    const Path64 rect_as_path_;
    const Point64 rect_mp_;
    Rect64 path_bounds_;
    std::deque<OutPt2> op_container_;
    OutPt2List results_;  // each path can be broken into multiples
    OutPt2List edges_[8]; // clockwise and counter-clockwise
    std::vector<Location> start_locs_;
    void CheckEdges();
    void TidyEdges(size_t idx, OutPt2List& cw, OutPt2List& ccw);
    void GetNextLocation(const Path64& path,
      Location& loc, size_t& i, size_t highI);
    OutPt2* Add(Point64 pt, bool start_new = false);
    void AddCorner(Location prev, Location curr);
    void AddCorner(Location& loc, bool isClockwise);
#ifdef USINGZ
  protected:
    ZCallback64 zCallback_ = nullptr;
    void SetZ(const Point64& e1bot, const Point64& e1top,
      const Point64& e2bot, const Point64& e2top, Point64& ip);
    void SetZ(const Point64& seg1, const Point64& seg2,
      Location edge_loc, Point64& ip);
#endif
  public:
    explicit RectClip64(const Rect64& rect) :
      rect_(rect),
      rect_as_path_(rect.AsPath()),
      rect_mp_(rect.MidPoint()) {}
#ifdef USINGZ
    void SetZCallback(ZCallback64 cb) { zCallback_ = cb; }
#endif
    Paths64 Execute(const Paths64& paths);
  };

  //------------------------------------------------------------------------------
  // RectClipLines64
  //------------------------------------------------------------------------------

  class RectClipLines64 : public RectClip64 {
  private:
    void ExecuteInternal(const Path64& path);
    Path64 GetPath(OutPt2*& op);
  public:
    explicit RectClipLines64(const Rect64& rect) : RectClip64(rect) {}
    Paths64 Execute(const Paths64& paths);
  };

} // Clipper2Lib namespace
#endif  // CLIPPER_RECTCLIP_H
