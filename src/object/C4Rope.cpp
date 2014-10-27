/*
 * OpenClonk, http://www.openclonk.org
 *
 * Copyright (c) 2012  Armin Burgmeier
 *
 * Portions might be copyrighted by other authors who have contributed
 * to OpenClonk.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * See isc_license.txt for full license and disclaimer.
 *
 * "Clonk" is a registered trademark of Matthes Bender.
 * See clonk_trademark_license.txt for full license.
 */

#include <queue>

#include <C4Include.h>
#include <C4Landscape.h>
#include <C4Rope.h>

#define C4ROPE_DRAW_DEBUG

namespace
{
	struct Vertex {
		Vertex() {}
		Vertex(float x, float y): x(x), y(y) {}

		float x;
		float y;
	};

	struct DrawVertex: Vertex {
		float u;
		float v;
	};

	// TODO: If the sqrts become a performance bottleneck we could also use an
	// approximation which works without Sqrt, cf. http://www.azillionmonkeys.com/qed/sqroot.html
	C4Real Len(C4Real dx, C4Real dy)
	{
		// Prevent possible overflow
		if(Abs(dx) > 120 || Abs(dy) > 120)
			return itofix(SqrtI(fixtoi(dx)*fixtoi(dx) + fixtoi(dy)*fixtoi(dy)));// ftofix(sqrt(fixtoi(dx)*fixtoi(dx) + fixtoi(dy)*fixtoi(dy)));
		else
			return Sqrt(dx*dx + dy*dy);//ftofix(sqrt(fixtof(dx*dx + dy*dy)));
	}

	// For use in initializer list
	C4Real ObjectDistance(C4Object* first, C4Object* second)
	{
		C4Real dx = second->fix_x - first->fix_x;
		C4Real dy = second->fix_y - first->fix_y;
		return Len(dx, dy);
	}

	// Helper function for Draw: determines vertex positions for one segment
	void VertexPos(Vertex& out1, Vertex& out2, Vertex& out3, Vertex& out4,
		             const Vertex& v1, const Vertex& v2, float w)
	{
		// This is for graphics only, so plain sqrt() is OK.
		const float l = sqrt( (v1.x - v2.x)*(v1.x - v2.x) + (v1.y - v2.y)*(v1.y - v2.y));

		out1.x = v1.x + w/2.0f * (v1.y - v2.y) / l;
		out1.y = v1.y - w/2.0f * (v1.x - v2.x) / l;
		out2.x = v1.x - w/2.0f * (v1.y - v2.y) / l;
		out2.y = v1.y + w/2.0f * (v1.x - v2.x) / l;
		out3.x = v2.x + w/2.0f * (v1.y - v2.y) / l;
		out3.y = v2.y - w/2.0f * (v1.x - v2.x) / l;
		out4.x = v2.x - w/2.0f * (v1.y - v2.y) / l;
		out4.y = v2.y + w/2.0f * (v1.x - v2.x) / l;
	}

	// Copied from StdGL.cpp... actually the rendering code should be moved
	// there so we don't need to duplicate it here.
	bool ApplyZoomAndTransform(float ZoomX, float ZoomY, float Zoom, C4BltTransform* pTransform)
	{
		// Apply zoom
		glTranslatef(ZoomX, ZoomY, 0.0f);
		glScalef(Zoom, Zoom, 1.0f);
		glTranslatef(-ZoomX, -ZoomY, 0.0f);

		// Apply transformation
		if (pTransform)
		{
			const GLfloat transform[16] = { pTransform->mat[0], pTransform->mat[3], 0, pTransform->mat[6], pTransform->mat[1], pTransform->mat[4], 0, pTransform->mat[7], 0, 0, 1, 0, pTransform->mat[2], pTransform->mat[5], 0, pTransform->mat[8] };
			glMultMatrixf(transform);

			// Compute parity of the transformation matrix - if parity is swapped then
			// we need to cull front faces instead of back faces.
			const float det = transform[0]*transform[5]*transform[15]
			                  + transform[4]*transform[13]*transform[3]
			                  + transform[12]*transform[1]*transform[7]
			                  - transform[0]*transform[13]*transform[7]
			                  - transform[4]*transform[1]*transform[15]
			                  - transform[12]*transform[5]*transform[3];
			return det > 0;
		}

		return true;
	}

	struct point_t { point_t(int x_, int y_): x(x_), y(y_) {} int x, y; };
	struct point_p { point_p(int x_, int y_, unsigned int f_): x(x_), y(y_), f(f_) {} int x, y; unsigned int f; };
	struct grid_t { int x, y; int pix, piy; unsigned int g; };

	// priority queue ordering for point_p class
	struct priority_cmp {
		bool operator()(const point_p& first, const point_p& second) { return first.f > second.f; }
	};

	// strict weak ordering for point_t class
	struct closed_point_cmp {
		bool operator()(const point_t& first, const point_t& second) { if(first.x != second.x) return first.x < second.x; return first.y < second.y; }
	};

	// Finds a way via pathfinder and stores all waypoints in the given vector
	bool FindWay(int from_x, int from_y, int to_x, int to_y,
	             std::vector<point_t>& waypoints)
	{
		assert(!GBackSolid(from_x, from_y) && !GBackSolid(to_x, to_y));

		// OK, so this is not using the C4PathFinder because it gives funny results at times, such as:
		// Node 0: 269 - 198
		// Node 1: 0 - 63
		// Node 2: 271 - 199
		// The result is also not optimal, such as this one:
		// Node 0: 219 - 197
		// Node 1: 227 - 198
		// Node 2: 221 - 198
		// Where going to 222,198 would have been enough for Node 1.

		// Instead, we use an path finding algorithm on a small grid. This is
		// good enough for our case, since distances we search are
		// usually very small.
		int min_x = Min(from_x, to_x) - abs(to_x - from_x) / 2;
		int max_x = Max(from_x, to_x) + abs(to_x - from_x) / 2;
		int min_y = Min(from_y, to_y) - abs(to_y - from_y) / 2;
		int max_y = Max(from_y, to_y) + abs(to_y - from_y) / 2;

		// Make a grid of at least 21x21 pixels
		static const unsigned int GRID_N = 21u;
		if(static_cast<unsigned int>(max_x - min_x) < GRID_N) { min_x -= (GRID_N - (max_x - min_x))/2; max_x += (GRID_N - (max_x - min_x))/2; }
		if(static_cast<unsigned int>(max_y - min_y) < GRID_N) { min_y -= (GRID_N - (max_y - min_y))/2; max_y += (GRID_N - (max_y - min_y))/2; }
		assert(min_x < from_x && max_x > from_x);
		assert(min_y < from_y && max_y > from_y);

		// Build the list of X and Y positions, inserting two more
		// pixels for the start and end point (can be same as existing pixels).
		int xs[GRID_N + 2], ys[GRID_N + 2];
		int xi = 0, yi = 0;
		int fxi = -1, fyi = -1, txi = -1, tyi = -1;
		for(unsigned int i = 0; i < GRID_N; ++i)
		{
			const int x = min_x + i * (max_x - min_x) / (GRID_N - 1);
			if(fxi == -1 && x >= from_x) { fxi = xi; xs[xi++] = from_x; }
			if(txi == -1 && x >= to_x) { txi = xi; xs[xi++] = to_x; }
			xs[xi++] = x;

			const int y = min_y + i * (max_y - min_y) / (GRID_N - 1);
			if(fyi == -1 && y >= from_y) { fyi = yi; ys[yi++] = from_y; }
			if(tyi == -1 && y >= to_y) { tyi = yi; ys[yi++] = to_y; }
			ys[yi++] = y;
		}

		// Build the grid itself
		grid_t grid[GRID_N + 2][GRID_N + 2];
		for(unsigned int x = 0; x < GRID_N + 2; ++x)
		{
			for(unsigned int y = 0; y < GRID_N + 2; ++y)
			{
				grid[x][y].x = xs[x];
				grid[x][y].y = ys[y];
				grid[x][y].pix = -1;
				grid[x][y].piy = -1;
				grid[x][y].g = 0;
			}
		}

		assert(grid[fxi][fyi].x == from_x && grid[fxi][fyi].y == from_y);
		assert(grid[txi][tyi].x == to_x && grid[txi][tyi].y == to_y);

		// Now scan the pixel grid which is (almost) evenly distributed in the
		// rectangle min_x,min_y/max_x,max_y for a path from from_x,from_y to to_x,to_y.
		// This is an A* implementation.
		const grid_t& target = grid[txi][tyi];
		std::priority_queue<point_p, std::vector<point_p>, priority_cmp> open;
		std::set<point_t, closed_point_cmp> closed; // Note the point_t entries are used as indices into the grid
		open.push(point_p(fxi, fyi, 0 /* irrelevant */));
		while(!open.empty())
		{
			point_p cur = open.top();
			open.pop();

			// Avoid seeing one node twice in a suboptimal way
			if(closed.find(point_t(cur.x, cur.y)) != closed.end())
				continue;

			// Found destination
			if(cur.x == txi && cur.y == tyi) break;

			// current point in grid
			const grid_t& cur_grid = grid[cur.x][cur.y];

			// expand
			const int adj_c[8][2] = { { cur.x - 1, cur.y - 1 },
			                        { cur.x + 0, cur.y - 1 },
			                        { cur.x + 1, cur.y - 1 },
			                        { cur.x - 1, cur.y + 0 },
			                        { cur.x + 1, cur.y + 0 },
			                        { cur.x - 1, cur.y + 1 },
			                        { cur.x + 0, cur.y + 1 },
			                        { cur.x + 1, cur.y + 1 } };
			for(int n = 0; n < 8; ++n)
			{
				const point_t adj(adj_c[n][0], adj_c[n][1]);
				if(adj.x < 0 || static_cast<unsigned int>(adj.x) > GRID_N+1 ||
				   adj.y < 0 || static_cast<unsigned int>(adj.y) > GRID_N+1) continue;
				if(closed.find(adj) != closed.end()) continue;
				grid_t& adj_grid = grid[adj.x][adj.y];
				if(!PathFree(cur_grid.x, cur_grid.y, adj_grid.x, adj_grid.y)) continue;

				const unsigned int tentative_g = cur_grid.g + Distance(cur_grid.x, cur_grid.y, adj_grid.x, adj_grid.y);
				if(adj_grid.pix != -1 && adj_grid.piy != -1 && tentative_g >= adj_grid.g) continue; 

				// Note that we might insert the element twice into the open list
				// here if there is already a less optimal way to adj. That's OK,
				// we ignore the second time at the beginning of the outer loop.
				adj_grid.pix = cur.x;
				adj_grid.piy = cur.y;
				adj_grid.g = tentative_g;
				open.push(point_p(adj.x, adj.y, tentative_g + Distance(adj_grid.x, adj_grid.y, target.x, target.y)));
			}

			closed.insert(point_t(cur.x, cur.y));
		}

		if(target.pix == -1 || target.piy == -1) return false;

		// Now reconstruct the path, omitting intermediate points
		// where possible
		std::vector<point_t> points;
		points.push_back(point_t(target.x, target.y));
		const grid_t* grid_p = &target;
		while(grid_p->pix != -1 && grid_p->piy != -1)
		{
			const grid_t* old_p = grid_p;
			grid_p = &grid[grid_p->pix][grid_p->piy];

			if(!PathFree(points.back().x, points.back().y, grid_p->x, grid_p->y))
			{
				assert(points.back().x != old_p->x || points.back().y != old_p->y);
				points.push_back(point_t(old_p->x, old_p->y));
			}
		}
		assert(grid_p == &grid[fxi][fyi]);
		assert(PathFree(points.back().x, points.back().y, grid_p->x, grid_p->y));
		points.push_back(point_t(grid_p->x, grid_p->y));

		// Reverse and attach to output
		for(std::vector<point_t>::const_reverse_iterator iter = points.rbegin(); iter != points.rend(); ++iter)
			waypoints.push_back(*iter);
		return true;
	}

	// Find a point on a line, with some tolerance to account for
	// rounding intermediate pixel positions. prev_x,prev_y is set to
	// the point just before the found point, and coll_x,coll_y is set
	// to the first point fulfilling the stopper condition.
	enum { STOP_AND, STOP_OR };
	template<typename StopperT>
	bool FindPointOnLine(int from_x, int from_y, int to_x, int to_y,
                             int* prev_x, int* prev_y, int* coll_x, int* coll_y,
	                     const StopperT& Stopper = StopperT())
	{
		int px = from_x;
		int py = from_y;
		const int max_p = Max(abs(to_x - from_x), abs(to_y - from_y));
		for(int i = 1; i <= max_p; ++i)
		{
			const int inter_x = from_x + i * (to_x - from_x) / max_p;
			const int inter_y = from_y + i * (to_y - from_y) / max_p;

			const int inter2_x = (abs(to_x - from_x) < abs(to_y - from_y) && i * (to_x - from_x) % max_p != 0) ? inter_x + Sign(to_x - from_x) : inter_x;
			const int inter2_y = (abs(to_x - from_x) > abs(to_y - from_y) && i * (to_y - from_y) % max_p != 0) ? inter_y + Sign(to_y - from_y) : inter_y;

			// The first and last point must be fixed
			assert( (inter_x == inter2_x && inter_y == inter2_y) || (i != 0 && i != max_p));

			if(Stopper(inter_x, inter_y))
			{
				if(StopperT::policy == STOP_OR || Stopper(inter2_x, inter2_y))
				{
					if(coll_x) *coll_x = inter_x;
					if(coll_y) *coll_y = inter_y;
					if(prev_x) *prev_x = px;
					if(prev_y) *prev_y = py;
					return true;
				}

				px = inter2_x;
				py = inter2_y;
			}
			else
			{
				if(StopperT::policy == STOP_OR && Stopper(inter2_x, inter2_y))
				{
					if(coll_x) *coll_x = inter2_x;
					if(coll_y) *coll_y = inter2_y;
					if(prev_x) *prev_x = px;
					if(prev_y) *prev_y = py;
					return true;
				}

				px = inter_x;
				py = inter_y;
			}
		}

		return false;
	}

	struct StopAtSolid {
		static const int policy = STOP_AND;
		bool operator()(int x, int y) const { return GBackSolid(x, y); }
	};

	struct StopAtPathBlocked {
		static const int policy = STOP_OR;
		StopAtPathBlocked(int to_x, int to_y): tox(to_x), toy(to_y) {}
		bool operator()(int x, int y) const { return !GBackSolid(x, y) && !PathFree(x, y, tox, toy); }
		const int tox, toy;
	};

	struct StopAt3PathFree {
		static const int policy = STOP_OR;
		StopAt3PathFree(int to1_x, int to1_y, int to2_x, int to2_y, int to3_x, int to3_y):
			to1x(to1_x), to1y(to1_y), to2x(to2_x), to2y(to2_y), to3x(to3_x), to3y(to3_y) {}
		bool operator()(int x, int y) const {
			return PathFree(x, y, to1x, to1y) && PathFree(x, y, to2x, to2y) && PathFree(x, y, to3x, to3y);
		}
		const int to1x, to1y, to2x, to2y, to3x, to3y;
	};
}

C4RopeElement::C4RopeElement(C4Object* obj, bool fixed):
	Fixed(fixed), x(Fix0), y(Fix0), oldx(obj->fix_x), oldy(obj->fix_y),
	vx(Fix0), vy(Fix0), fx(Fix0), fy(Fix0),
	rx(Fix0), ry(Fix0), rdt(Fix0), fcx(Fix0), fcy(Fix0),
	Next(NULL), Prev(NULL), FirstLink(NULL), LastLink(NULL),
	Object(obj), LastContactVertex(-1)
{
}

C4RopeElement::C4RopeElement(C4Real x, C4Real y, C4Real m, bool fixed):
	Fixed(fixed), x(x), y(y), oldx(x), oldy(y), vx(Fix0), vy(Fix0), m(m),
	fx(Fix0), fy(Fix0), rx(Fix0), ry(Fix0), rdt(Fix0), fcx(Fix0), fcy(Fix0),
	Next(NULL), Prev(NULL), FirstLink(NULL), LastLink(NULL),
	Object(NULL), LastContactVertex(-1)
{
}

C4RopeElement::~C4RopeElement()
{
	for(C4RopeLink* link = FirstLink, *next; link != NULL; link = next)
	{
		next = link->Next;
		delete link;
	}
}

void C4RopeElement::AddForce(C4Real x, C4Real y)
{
	fx += x;
	fy += y;

#ifdef C4ROPE_DRAW_DEBUG
	fcx += x;
	fcy += y;
#endif
}

C4Real C4RopeElement::GetTargetX() const
{
	// TODO: Prevent against object changes: Reset when Target changes wrt
	// to the object LastContactVertex refers to.
	if(LastContactVertex == -1) return GetX();

	C4Object* obj = Object;
	while(obj->Contained)
		obj = obj->Contained;
	return GetX() + itofix(obj->Shape.VtxX[LastContactVertex]);
}

C4Real C4RopeElement::GetTargetY() const
{
	// TODO: Prevent against object changes: Reset when Target changes wrt
	// to the object LastContactVertex refers to.
	if(LastContactVertex == -1) return GetY();

	C4Object* obj = Object;
	while(obj->Contained)
		obj = obj->Contained;
	return GetY() + itofix(obj->Shape.VtxY[LastContactVertex]);
}

void C4RopeElement::ScanAndInsertLinks(C4RopeElement* from, C4RopeElement* to, int from_x, int from_y, int to_x, int to_y, int link_x, int link_y)
{
	// Rope buried
	if(GBackSolid(from_x, from_y) || GBackSolid(to_x, to_y)) return;

	// Nothing to do
	if(PathFree(to_x, to_y, link_x, link_y)) return;

	// Sanity check, this might be violated if the landscape changed.
	// In that case the rope might end up buried in earth, in which case we
	// do not want to do any maneuvering around, but the rope is just stuck.
	// note that this should never fail as long as the landscape remains unchanged.
	if(!PathFree(from_x, from_y, link_x, link_y)) return;

	// Find the position between from_x,from_y, and to_x,to_y
	// where there is no path free to link_x,link_y anymore.
	std::vector<point_t> remaining_waypoints;
	if(PathFree(from_x, from_y, to_x, to_y))
	{
		remaining_waypoints.push_back(point_t(from_x, from_y));
		remaining_waypoints.push_back(point_t(to_x, to_y));
	}
	else
	{
		// This does not necessarily mean that the rope element moved
		// through solid wall, it can also be one of the following:
		// * object movement behaving slightly differently from rope
		// movement, especially for attached movement.
		// * for non-object rope elements, more than one movement
		// iteration is being performed per frame, whereas link
		// scanning is only being performed once per frame. So the
		// element might have moved around a corner within one frame.
		// * note that this is likely not caused by changes to the
		// landscape, because this would already cause the
		// "sanity check" above to fail, which is a precondition to
		// the algorithm.

		// The idea is to run a pathfinder to find a path. This
		// should work well for around-the-corner movement. If the
		// path found turns out to be ridicilously long, we bail.
		if(!FindWay(from_x, from_y, to_x, to_y, remaining_waypoints))
			return;

		unsigned int len = 0;
		const unsigned int direct_len = Distance(from_x, from_y, to_x, to_y);
		for(unsigned int i = 1; i < remaining_waypoints.size(); ++i)
		{
			const int wp1_x = remaining_waypoints[i-1].x;
			const int wp1_y = remaining_waypoints[i-1].y;
			const int wp2_x = remaining_waypoints[i].x;
			const int wp2_y = remaining_waypoints[i].y;

			assert(PathFree(wp1_x, wp1_y, wp2_x, wp2_y));
			len += Distance(wp1_x, wp1_y, wp2_x, wp2_y);
		}

		// Way too long...
		if(len > Max(15u, 2*direct_len)) return;
	}

	assert(remaining_waypoints.size() >= 2);
	assert(remaining_waypoints.begin()->x == from_x);
	assert(remaining_waypoints.begin()->y == from_y);
	assert(remaining_waypoints.rbegin()->x == to_x);
	assert(remaining_waypoints.rbegin()->y == to_y);

	// Now, the path between all waypoints (in the easiest case a straight
	// line from from_x,from_y to to_x,to_y) for the point at which the
	// path to link_x,link_y is lost.
	int prev_x = -1, prev_y = -1;
	int coll_x = -1, coll_y = -1;
	for(unsigned int i = 1; i < remaining_waypoints.size(); ++i)
	{
		const int wp1_x = remaining_waypoints[i-1].x;
		const int wp1_y = remaining_waypoints[i-1].y;
		const int wp2_x = remaining_waypoints[i].x;
		const int wp2_y = remaining_waypoints[i].y;
		assert(PathFree(wp1_x, wp1_y, wp2_x, wp2_y));

		if(FindPointOnLine(wp1_x, wp1_y, wp2_x, wp2_y,
	                           &prev_x, &prev_y, &coll_x, &coll_y,
	                           StopAtPathBlocked(link_x, link_y)))
		{
			// The waypoints are constructed such that the paths between them are free:
			assert(!GBackSolid(coll_x, coll_y));
			remaining_waypoints.erase(remaining_waypoints.begin(), remaining_waypoints.begin() + i);
			break;
		}
	}

	// Such a point must exist, since one precondition of this algorithm is
	// that PathFree(to_x, to_y, link_x, link_y) is false.
	assert(prev_x != -1 || prev_y != -1 || coll_x != -1 || coll_y != -1);

	// Now we have:
	// prev_x,prev_y: Last position between from_x,from_y and to_x,to_y
	// where the path to link_x,link_y is still free.
	// coll_x,coll_y: First position between from_x,from_y and to_x,to_y
	// where the path to link_x,link_y is no longer free.

	// Now the idea is to insert a new link somewhere on the line from
	// link_x,link_y to prev_x,prev_y (which is free!) such that
	// PathFree(coll_x, coll_y, insert_x, insert_y) holds and the point is
	// as close to link_x, link_y as possible. We check
	// PathFree(wp_x,wp_y,insert_x,insert_y) in addition since we didn't
	// do so before, and we also explicitely check
	// PathFree(link_x, link_y, insert_x, insert_y) to make sure that due
	// to the finite pixel grid the PathFree condition still holds.
	int insert_x, insert_y;
	const bool haveInsert = FindPointOnLine(link_x, link_y, prev_x, prev_y,
	                                        NULL, NULL, &insert_x, &insert_y,
	                                        StopAt3PathFree(link_x, link_y, remaining_waypoints[0].x, remaining_waypoints[0].y, coll_x, coll_y));
	if(!haveInsert)
	{
		// TODO: Note that even without changing landscape this can happen
		// when maneuvering around corners in the landscape. It would be
		// good if we could fix this -- in theory it should not happen.
	}
	else
	{
		InsertLink(from, to, insert_x, insert_y);
		for(unsigned int i = 0; i < remaining_waypoints.size() - 1; ++i)
			InsertLink(from, to, remaining_waypoints[i].x, remaining_waypoints[i].y);
	}
}

void C4RopeElement::InsertLink(C4RopeElement* from, C4RopeElement* to, int insert_x, int insert_y)
{
	assert(this == from || this == to);

	C4RopeLink* Link = new C4RopeLink;
	Link->x = itofix(insert_x);
	Link->y = itofix(insert_y);

	if(this == from)
	{
		assert(from->Next == to);

		Link->Next = from->FirstLink;
		if(from->FirstLink)
			from->FirstLink->Prev = Link;
		from->FirstLink = Link;
		Link->Prev = NULL;

		if(to->LastLink == NULL)
			to->LastLink = Link;
	}
	else
	{
		assert(to->Prev == from);

		Link->Prev = to->LastLink;
		if(to->LastLink)
			to->LastLink->Next = Link;
		to->LastLink = Link;
		Link->Next = NULL;

		if(from->FirstLink == NULL)
			from->FirstLink = Link;
	}
}

void C4RopeElement::RemoveFirstLink()
{
	assert(FirstLink != NULL);

	C4RopeLink* link = FirstLink;
	FirstLink = link->Next;
	if(Next && link->Next == NULL)
		Next->LastLink = NULL;
	if(FirstLink)
		FirstLink->Prev = NULL;

	delete link;
}

void C4RopeElement::RemoveLastLink()
{
	assert(LastLink != NULL);

	C4RopeLink* link = LastLink;
	LastLink = link->Prev;
	if(Prev && link->Prev == NULL)
		Prev->FirstLink = NULL;
	if(LastLink)
		LastLink->Next = NULL;

	delete link;
}

void C4RopeElement::Execute(const C4Rope* rope, C4Real dt)
{
	ResetForceRedirection(dt);

	// If attached object is contained, apply force to container
	C4Object* Target = Object;
	if(Target)
		while(Target->Contained)
			Target = Target->Contained;

	// Apply sticking friction
	if(!Target)
	{
		// Sticking friction: If a segment has contact with the landscape and it
		// is at rest then one needs to exceed a certain threshold force until it
		// starts moving.
		int ix = fixtoi(x);
		int iy = fixtoi(y);
		if(GBackSolid(ix+1, iy) || GBackSolid(ix-1, iy) || GBackSolid(ix, iy-1) || GBackSolid(ix, iy+1))
		{
			if(vx*vx + vy*vy < Fix1/4) // TODO: Threshold should be made a property
			{
				if(fx*fx + fy*fy < Fix1/4) // TODO: Threshold should be made a property
				{
					fx = fy = Fix0;
					vx = vy = Fix0;
					return;
				}
			}
		}
	}

	// Apply forces
	if(!Fixed)
	{
		if(!Target)
		{
			vx += dt * fx / m;
			vy += dt * fy / m;
		}
		else if( (Target->Category & C4D_StaticBack) == 0)
		{
			// Only apply xdir/ydir to targets if they are not attached
			if((Target->Action.t_attach & (CNAT_Left | CNAT_Right)) == 0)
				Target->xdir += dt * fx / Target->Mass;
			if((Target->Action.t_attach & (CNAT_Top | CNAT_Bottom)) == 0)
				Target->ydir += dt * fy / Target->Mass;
		}
	}
	fx = fy = Fix0;

	// Execute movement
	if(!Fixed)
	{
		// Compute old and new coordinates
		const int old_x = fixtoi(GetX());
		const int old_y = fixtoi(GetY());
		const int new_x = fixtoi(GetX() + dt * GetVx());
		const int new_y = fixtoi(GetY() + dt * GetVy());

		int prev_x, prev_y;
		const bool haveColl = FindPointOnLine(old_x, old_y, new_x, new_y,
		                                      &prev_x, &prev_y, NULL, NULL,
		                                      StopAtSolid());
		if(haveColl)
		{
			if(!Target)
			{
				x = itofix(prev_x);
				y = itofix(prev_y);
			}
			else
			{
				// For objects, x/y is only an offset to the
				// object position used for sub-frame movement,
				// since objects are otherwise only executed
				// once a frame.
				x = itofix(prev_x) - oldx;
				y = itofix(prev_y) - oldy;
			}

			// TODO: Apply this friction force perpendicular to surface normal

			// Apply friction force
			fx -= rope->GetOuterFriction() * GetVx(); fy -= rope->GetOuterFriction() * GetVy();
			// Force redirection so that not every single pixel on a
			// chunky landscape is an obstacle for the rope.
			SetForceRedirection(rope, 0, 0);
		}
		else
		{
			// Don't use new_x to keep subpixel precision
			x += dt * GetVx();
			y += dt * GetVy();
		}
	}

	if(Target)
	{
		// Object Force redirection if object has no contact attachment (if it
		// has then the procedure takes care of moving the object around
		// O(pixel) obstacles in the landscape).
		if(!Target->Action.t_attach && Target->xdir*Target->xdir + Target->ydir*Target->ydir >= Fix1)
		{
			// Check if the object has contact to the landscape
			//long iResult = 0;
			const DWORD dwCNATCheck = CNAT_Left | CNAT_Right | CNAT_Top | CNAT_Bottom;
			int iContactVertex = -1;
			for (int i = 0; i < Target->Shape.VtxNum; ++i)
				if(Target->Shape.GetVertexContact(i, dwCNATCheck, Target->GetX(), Target->GetY()))
					iContactVertex = i;

			if(iContactVertex != -1)
			{
				LastContactVertex = iContactVertex;
				SetForceRedirection(rope, Target->Shape.VtxX[iContactVertex], Target->Shape.VtxY[iContactVertex]);
			}
		}
	}
}

void C4RopeElement::ResetForceRedirection(C4Real dt)
{
	// countdown+reset force redirection
	if(rdt != Fix0)
	{
		if(dt > rdt)
		{
			rx = ry = Fix0;
			rdt = Fix0;
		}
		else
		{
			rdt -= dt;
		}
	}
}

void C4RopeElement::SetForceRedirection(const C4Rope* rope, int ox, int oy)
{
	SetForceRedirectionByLookAround(rope, ox, oy, GetVx(), GetVy(), itofix(5), itofix(75));

#if 0
	if(!SetForceRedirectionByLookAround(rope, ox, oy, GetVx(), GetVy(), itofix(5), itofix(15)))
		if(!SetForceRedirectionByLookAround(rope, ox, oy, GetVx(), GetVy(), itofix(5), itofix(30)))
			SetForceRedirectionByLookAround(rope, ox, oy, GetVx(), GetVy(), itofix(5), itofix(45));
			//SetForceRedirectionByLookAround(rope, ox, oy, GetVx(), GetVy(), itofix(5), itofix(60));
#endif
}

bool C4RopeElement::SetForceRedirectionByLookAround(const C4Rope* rope, int ox, int oy, C4Real dx, C4Real dy, C4Real l, C4Real angle)
{
	// The procedure is the following: we try manuevering around
	// the obstacle, either left or right.  Which way we take is determined by
	// checking whether or not there is solid material in a 75 degree angle
	// relative to the direction of movement.
	const C4Real Cos75 = Cos(angle);
	const C4Real Sin75 = Sin(angle);

	C4Real vx1 =  Cos75 * dx + Sin75 * dy;
	C4Real vy1 = -Sin75 * dx + Cos75 * dy;
	C4Real vx2 =  Cos75 * dx - Sin75 * dy;
	C4Real vy2 =  Sin75 * dx + Cos75 * dy;
	const C4Real v = Len(dx, dy);

	// TODO: We should check more than a single pixel. There's some more potential for optimization here.
	if(v != Fix0 && !GBackSolid(ox + fixtoi(GetX() + vx1*l/v), oy + fixtoi(GetY() + vy1*l/v)))
	//if(v != Fix0 && PathFree(ox + fixtoi(GetX()), oy + fixtoi(GetY()), ox + fixtoi(GetX() + vx1*l/v), oy + fixtoi(GetY() + vy1*l/v)))
		{ rx = vx1/v; ry = vy1/v; rdt = Fix1/4; } // Enable force redirection for 1/4th of a frame
	else if(v != Fix0 && !GBackSolid(ox + fixtoi(GetX() + vx2/v), oy + fixtoi(GetY() + vy2/v)))
	//else if(v != Fix0 && PathFree(ox + fixtoi(GetX()), oy + fixtoi(GetY()), ox + fixtoi(GetX() + vx2/v), oy + fixtoi(GetY() + vy2/v)))
		{ rx = vx2/v; ry = vy2/v; rdt = Fix1/4; } // Enable force redirection for 1/4th of a frame
	else
		return false;
	return true;
}

C4Rope::C4Rope(C4PropList* Prototype, C4Object* first_obj, C4Object* second_obj, C4Real segment_length, C4DefGraphics* graphics):
	C4PropListNumbered(Prototype), Width(5.0f), Graphics(graphics), SegmentCount(fixtoi(ObjectDistance(first_obj, second_obj)/segment_length)),
	l(segment_length), k(Fix1*20 /* TODO: Too much spring like when hanging in air */), mu(Fix1*3), eta(Fix1*4), NumIterations(10),
	FrontAutoSegmentation(Fix0), BackAutoSegmentation(Fix0), FrontPull(Fix0), BackPull(Fix0)
{
	if(!PathFree(first_obj->GetX(), first_obj->GetY(), second_obj->GetX(), second_obj->GetY()))
		throw C4RopeError("Path between objects is blocked");
	if(Graphics->Type != C4DefGraphics::TYPE_Bitmap)
		throw C4RopeError("Can only use bitmap as rope graphics");

	Front = new C4RopeElement(first_obj, false);
	Back = new C4RopeElement(second_obj, false);
	
	const C4Real m(Fix1); // TODO: This should be a property

	C4RopeElement* prev_seg = Front;
	for(int32_t i = 0; i < SegmentCount; ++i)
	{
		// Create new element
		C4Real seg_x = first_obj->fix_x + (second_obj->fix_x - first_obj->fix_x) * (i+1) / (SegmentCount+1);
		C4Real seg_y = first_obj->fix_y + (second_obj->fix_y - first_obj->fix_y) * (i+1) / (SegmentCount+1);
		C4RopeElement* seg = new C4RopeElement(seg_x, seg_y, m, false);

		// Link it
		seg->Prev = prev_seg;
		prev_seg->Next = seg;
		prev_seg = seg;
	}

	// Link back segment
	prev_seg->Next = Back;
	Back->Prev = prev_seg;
}

C4Rope::~C4Rope()
{
	for(C4RopeElement* cur = Front, *next; cur != NULL; cur = next)
	{
		next = cur->Next;
		delete cur;
	}
}

bool C4Rope::IsStuck() const
{
	int prevx, prevy;
	for(C4RopeElement* elem = Front; elem != NULL; elem = elem->Next)
	{
		const int x = fixtoi(elem->GetX());
		const int y = fixtoi(elem->GetY());
		if((elem != Front) && !PathFree(prevx, prevy, x, y))
			return true;
		prevx = x; prevy = y;

		for(C4RopeLink* link = elem->FirstLink; link != NULL; link = link->Next)
		{
			const int x = fixtoi(link->x);
			const int y = fixtoi(link->y);
			if(!PathFree(prevx, prevy, x, y))
				return true;
			prevx = x; prevy = y;
		}
	}

	return false;
}

C4Real C4Rope::GetL(const C4RopeElement* prev, const C4RopeElement* next) const
{
	// Normally the segment length is fixed at l, however if auto segmentation
	// is enabled then the first or last segments can be shorter.
	const C4Real dx = next->GetX() - prev->GetX();
	const C4Real dy = next->GetY() - prev->GetY();

	if(FrontAutoSegmentation > Fix0)
		if(prev == Front || next == Front)
			return Min(itofix(5), Len(dx, dy));
	if(BackAutoSegmentation > Fix0)
		if(prev == Back || next == Back)
			return Min(itofix(5), Len(dx, dy));

	return l;
}

void C4Rope::DoAutoSegmentation(C4RopeElement* fixed, C4RopeElement* first, C4Real max)
{
	// TODO: Should add a timeout to prevent oscillations: After one segment was
	// inserted do not allow segments to be removed in the same frame or couple
	// of frames, and vice versa. This gives the system a chance to get into
	// equilibrium before continuing with auto-segmentation.

	// Auto segmentation enabled?
	if(max > Fix0)
	{
		const C4Real dx = first->GetX() - fixed->GetX();
		const C4Real dy = first->GetY() - fixed->GetY();
		const C4Real lf = dx*dx+dy*dy;

		if(lf > l*l*itofix(15,10)*itofix(15,10) && l * SegmentCount < max)
		{
			const C4Real x = fixed->GetX() + itofix(5,10)*dx;
			const C4Real y = fixed->GetY() + itofix(5,10)*dy;
			C4RopeElement* new_elem = new C4RopeElement(x, y, first->GetMass(), false);
			new_elem->vx = (fixed->GetVx() + first->GetVx())/itofix(2);
			new_elem->vy = (fixed->GetVy() + first->GetVy())/itofix(2);

			// Link the new element
			if(fixed->Next == first)
			{
				new_elem->Prev = fixed;
				new_elem->Next = first;
				fixed->Next = new_elem;
				first->Prev = new_elem;
			}
			else
			{
				new_elem->Prev = first;
				new_elem->Next = fixed;
				fixed->Prev = new_elem;
				first->Next = new_elem;
			}
			++SegmentCount;
		}
		else if(SegmentCount > 0) // Rope cannot be shorter than just Beginning and End segment
		{
			// To find out whether we can shorten the rope we do the following:
			// We go through all elements and if at some point the nominal rope length
			// is shorter than the distance between that element and the fixpoint
			// and the path between the two is free, then the rope is shortened.
			unsigned int i = 1;
			for(C4RopeElement* cur = first; cur != NULL; cur = (fixed->Next == first ? cur->Next : cur->Prev), ++i)
			{
				// We use integers, not reals here, to protect for overflows. This works
				// because these numbers are large enough so we don't need to take care
				// about subpixel precision.
				const unsigned int nd = fixtoi(l*itofix(i));

				const unsigned int dx = fixtoi(cur->GetX() - fixed->GetX());
				const unsigned int dy = fixtoi(cur->GetY() - fixed->GetY());
				const unsigned int d2 = dx*dx+dy*dy;

				if(d2 > nd*nd*15/10*15/10)
					break;
				else if(d2 < nd*nd*8/10*8/10)
				{
					// TODO: Check whether all elements have PathFree, and stop if one hasn't?
					if(PathFree(fixtoi(fixed->GetX()), fixtoi(fixed->GetY()), fixtoi(cur->GetX()), fixtoi(cur->GetY())))
					{
						C4RopeElement* second = ((fixed->Next == first) ? first->Next : first->Prev);
						assert(second != NULL);

						// Remove first, relink fixed and second
						C4RopeElement* Del = first;

						if(fixed->Next == first)
						{
							fixed->Next = second;
							second->Prev = fixed;
						}
						else
						{
							fixed->Prev = second;
							second->Next = fixed;
						}

						delete Del;
						--SegmentCount;
					}

					break;
				}
			}
		}
	}
}

void C4Rope::Solve(C4RopeElement* prev, C4RopeElement* next)
{
	// Rope forces
	const C4Real dx = prev->GetX() - next->GetX();
	const C4Real dy = prev->GetY() - next->GetY();

	if(dx != Fix0 || dy != Fix0) //dx*dx + dy*dy > Fix0)
	{
		// Get segment length between prev and next
		const C4Real l = GetL(prev, next);
		// Compute forces between these points.
		C4Real dx1, dy1, dx2, dy2, d;
		if(prev->FirstLink)
		{
			const C4Real l1x = prev->FirstLink->x;
			const C4Real l1y = prev->FirstLink->y;
			const C4Real l2x = next->LastLink->x;
			const C4Real l2y = next->LastLink->y;

			d = Len(prev->GetX() - l1x, prev->GetY() - l1y)
			  + Len(next->GetX() - l2x, next->GetY() - l2y);
			for(C4RopeLink* l = prev->FirstLink; l->Next != NULL; l = l->Next)
				d += Len(l->x - l->Next->x, l->y - l->Next->y);

			dx1 = l1x - prev->GetX();
			dy1 = l1y - prev->GetY();
			dx2 = l2x - next->GetX();
			dy2 = l2y - next->GetY();

			const C4Real d1 = Len(dx1, dy1);
			const C4Real d2 = Len(dx2, dy2);
			if(d1 != Fix0)
				{ dx1 /= d1; dy1 /= d1; }
			else
				{ dx1 = dy1 = Fix0; }

			if(d2 != Fix0)
				{ dx2 /= d2; dy2 /= d2; }
			else
				{ dx2 = dy2 = Fix0; }
		}
		else
		{
			d = Len(dx, dy);
			if(d != Fix0)
				{ dx1 = -(dx / d); dy1 = -(dy / d); }
			else
				{ dx1 = 0; dx2 = 0; }

			dx2 = -dx1;
			dy2 = -dy1;
		}

		if(ApplyRepulsive || d > l)
		{
			if(prev->rdt != Fix0) { dx1 = prev->rx; dy1 = prev->ry; }
			if(next->rdt != Fix0) { dx2 = next->rx; dy2 = next->ry; }

			prev->AddForce(dx1 * k * (d - l), dy1 * k * (d - l));
			next->AddForce(dx2 * k * (d - l), dy2 * k * (d - l));
		}
	}

	// Inner friction
	// TODO: This is very sensitive to numerical instabilities for mid-to-high
	// eta values. We might want to prevent a sign change of either F or V induced
	// by this factor.
	// TODO: Don't apply inner friction for segments connected to fixed rope ends?
	C4Real fx = (prev->GetVx() - next->GetVx()) * eta;
	C4Real fy = (prev->GetVy() - next->GetVy()) * eta;
	prev->AddForce(-fx, -fy);
	next->AddForce(fx, fy);

	// Could add air/water friction here

	// TODO: Apply gravity separately. This is applied twice now for
	// non-end rope segments!

	// Don't apply gravity to objects since it's applied already in C4Object execution.
	prev->AddForce(Fix0, (prev->GetObject() ? Fix0 : prev->GetMass() * ::Landscape.Gravity/5));
	next->AddForce(Fix0, (prev->GetObject() ? Fix0 : next->GetMass() * ::Landscape.Gravity/5));
}

void C4Rope::Execute()
{
	C4Real dt = itofix(1, NumIterations);
	for(unsigned int i = 0; i < NumIterations; ++i)
	{
		// Execute auto-segmentation
		DoAutoSegmentation(Front, Front->Next, FrontAutoSegmentation);
		DoAutoSegmentation(Back, Back->Prev, BackAutoSegmentation);

		// Reset previous forces
#ifdef C4ROPE_DRAW_DEBUG
		for(C4RopeElement* cur = Front; cur != NULL; cur = cur->Next)
			cur->fcx = cur->fcy = Fix0;
#endif

		// Compute inter-rope forces
		for(C4RopeElement* cur = Front; cur->Next != NULL; cur = cur->Next)
			Solve(cur, cur->Next);

		// Front/BackPull
		if(FrontPull != Fix0)
		{
			// Pull at all elements that are near to the front element
			C4RopeElement* cur = Front;
			C4Real d;
			do
			{
				cur = cur->Next;
				const C4Real dx = cur->GetX() - Front->GetX();
				const C4Real dy = cur->GetY() - Front->GetY();
				d = Len(dx, dy);
				if(d != Fix0)
					cur->AddForce(-dx/d * FrontPull, -dy/d * FrontPull);
			} while(cur->Next != NULL && d < itofix(5,10*l));
		}

		if(BackPull != Fix0)
		{
			// Pull at all elements that are near to the back element
			C4RopeElement* cur = Back;
			C4Real d;
			do
			{
				cur = cur->Prev;
				const C4Real dx = cur->GetX() - Back->GetX();
				const C4Real dy = cur->GetY() - Back->GetY();
				d = Len(dx, dy);
				if(d != Fix0)
					cur->AddForce(-dx/d * BackPull, -dy/d * BackPull);
			} while(cur->Prev != NULL && d < itofix(5,10*l));
		}

		// Apply forces
		for(C4RopeElement* cur = Front; cur != NULL; cur = cur->Next)
			cur->Execute(this, dt);
	}

	// Insert/remove links between rope elements. We rely that object movement
	// is executed before rope movement at this point, so this needs to stay in
	// sync with C4Game::Execute().
	for(C4RopeElement* cur = Front; cur != NULL; cur = cur->Next)
	{
		// Note that for the previous link we use the new coordinates, but for following links the new coordinates.
		// This guarantees that we will find a free path everytime.
		if(cur->Prev)
			cur->ScanAndInsertLinks(cur->Prev, cur, fixtoi(cur->oldx), fixtoi(cur->oldy), fixtoi(cur->GetX()), fixtoi(cur->GetY()), fixtoi(cur->LastLink ? cur->LastLink->x : cur->Prev->GetX()), fixtoi(cur->LastLink ? cur->LastLink->y : cur->Prev->GetY()));
		if(cur->Next)
			cur->ScanAndInsertLinks(cur, cur->Next, fixtoi(cur->oldx), fixtoi(cur->oldy), fixtoi(cur->GetX()), fixtoi(cur->GetY()), fixtoi(cur->FirstLink ? cur->FirstLink->x : cur->Next->oldx), fixtoi(cur->FirstLink ? cur->FirstLink->y : cur->Next->oldy));

		while(cur->Prev && cur->LastLink && PathFree(fixtoi(cur->GetX()), fixtoi(cur->GetY()), fixtoi(cur->LastLink->Prev ? cur->LastLink->Prev->x : cur->Prev->GetX()), fixtoi(cur->LastLink->Prev ? cur->LastLink->Prev->y : cur->Prev->GetY())))
			cur->RemoveLastLink();
		while(cur->Next && cur->FirstLink && PathFree(fixtoi(cur->GetX()), fixtoi(cur->GetY()), fixtoi(cur->FirstLink->Next ? cur->FirstLink->Next->x : cur->Next->GetX()), fixtoi(cur->FirstLink->Next ? cur->FirstLink->Next->y : cur->Next->GetY())))
			cur->RemoveFirstLink();
	}

	// Update old coordinates for next iteration.
	for(C4RopeElement* cur = Front; cur != NULL; cur = cur->Next)
	{
		if(cur->Object != NULL)
		{
			cur->oldx = cur->Object->fix_x;
			cur->oldy = cur->Object->fix_y;
			// Reset offset
			cur->x = Fix0;
			cur->y = Fix0;
			cur->vx = Fix0;
			cur->vy = Fix0;
		}
		else
		{
			cur->oldx = cur->x;
			cur->oldy = cur->y;
		}
	}
}

void C4Rope::ClearPointers(C4Object* obj)
{
	for(C4RopeElement* cur = Front; cur != NULL; cur = cur->Next)
	{
		if(cur->GetObject() == obj)
		{
			cur->x = obj->fix_x;
			cur->y = obj->fix_y;
			cur->vx = obj->xdir;
			cur->vy = obj->ydir;
			cur->m = obj->Mass;

			cur->Object = NULL;
		}
	}
}

// TODO: Move this to StdGL
void C4Rope::Draw(C4TargetFacet& cgo, C4BltTransform* pTransform)
{
#ifndef C4ROPE_DRAW_DEBUG
	Vertex Tmp[4];
	DrawVertex* Vertices = new DrawVertex[SegmentCount*2+4]; // TODO: Use a vbo and map it into memory instead?

	VertexPos(Vertices[0], Vertices[1], Tmp[0], Tmp[1],
	          Vertex(fixtof(Front->GetX()), fixtof(Front->GetY())),
	          Vertex(fixtof(Front->Next->GetX()), fixtof(Front->Next->GetY())), Width);

	Vertices[0].u = 0.0f;
	Vertices[0].v = 0.0f;
	Vertices[1].u = 1.0f;
	Vertices[1].v = 0.0f;

	const float rsl = 1.0f/Width * Graphics->GetBitmap()->Wdt / Graphics->GetBitmap()->Hgt; // rope segment length mapped to Gfx bitmap
	float accl = 0.0f;

	unsigned int i = 2;
	bool parity = true;
	for(C4RopeElement* cur = Front->Next; cur->Next != NULL; cur = cur->Next, i += 2)
	{
		Vertex v1(fixtof(cur->GetX()),
		          fixtof(cur->GetY()));
		Vertex v2(fixtof(cur->Next->GetX()),// ? cur->Next->GetX() : Back->GetX()), 
		          fixtof(cur->Next->GetY()));// ? cur->Next->GetY() : Back->GetY()));
		Vertex v3(fixtof(cur->Prev->GetX()),// ? cur->Prev->GetX() : Front->GetX()),
		          fixtof(cur->Prev->GetY()));// ? cur->Prev->GetY() : Front->GetY()));

		//const C4Real l = GetL(cur->Prev, cur);
		//const float rsl = fixtof(l)/fixtof(Width) * Graphics->GetBitmap()->Wdt / Graphics->GetBitmap()->Hgt; // rope segment length mapped to Gfx bitmap

		// Parity -- parity swaps for each pointed angle (<90 deg)
		float cx = v1.x - v3.x;
		float cy = v1.y - v3.y;
		float ex = v1.x - v2.x;
		float ey = v1.y - v2.y;
		
		// TODO: Another way to draw this would be to insert a "pseudo" segment so that there are no pointed angles at all
		if(cx*ex+cy*ey > 0)
			parity = !parity;

		// Obtain vertex positions
		if(parity)
			VertexPos(Tmp[2], Tmp[3], Vertices[i+2], Vertices[i+3], v1, v2, Width);
		else
			VertexPos(Tmp[3], Tmp[2], Vertices[i+3], Vertices[i+2], v1, v2, Width);

		Tmp[2].x = (Tmp[0].x + Tmp[2].x)/2.0f;
		Tmp[2].y = (Tmp[0].y + Tmp[2].y)/2.0f;
		Tmp[3].x = (Tmp[1].x + Tmp[3].x)/2.0f;
		Tmp[3].y = (Tmp[1].y + Tmp[3].y)/2.0f;

		// renormalize
		float dx = Tmp[3].x - Tmp[2].x;
		float dy = Tmp[3].y - Tmp[2].y;
		float dx2 = Vertices[i-1].x - Vertices[i-2].x;
		float dy2 = Vertices[i-1].y - Vertices[i-2].y;
		const float d = (dx2*dx2+dy2*dy2)/(dx*dx2+dy*dy2);
		Vertices[i  ].x = ( (Tmp[2].x + Tmp[3].x)/2.0f) - BoundBy((Tmp[3].x - Tmp[2].x)*d, -Width, Width)/2.0f;
		Vertices[i  ].y = ( (Tmp[2].y + Tmp[3].y)/2.0f) - BoundBy((Tmp[3].y - Tmp[2].y)*d, -Width, Width)/2.0f;
		Vertices[i+1].x = ( (Tmp[2].x + Tmp[3].x)/2.0f) + BoundBy((Tmp[3].x - Tmp[2].x)*d, -Width, Width)/2.0f;
		Vertices[i+1].y = ( (Tmp[2].y + Tmp[3].y)/2.0f) + BoundBy((Tmp[3].y - Tmp[2].y)*d, -Width, Width)/2.0f;

		accl += fixtof(GetL(cur->Prev, cur));
		Vertices[i].u = 0.0f; //parity ? 0.0f : 1.0f;
		Vertices[i].v = accl * rsl;
		Vertices[i+1].u = 1.0f; //parity ? 1.0f : 0.0f;
		Vertices[i+1].v = accl * rsl;

		Tmp[0] = Vertices[i+2];
		Tmp[1] = Vertices[i+3];
	}

	accl += fixtof(GetL(Back->Prev, Back));
	Vertices[i].u = 0.0f; //parity ? 0.0f : 1.0f;
	Vertices[i].v = accl * rsl;
	Vertices[i+1].u = 1.0f; //parity ? 1.0f : 0.0f;
	Vertices[i+1].v = accl * rsl;

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, (*Graphics->GetBitmap()->ppTex)->texName);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glVertexPointer(2, GL_FLOAT, sizeof(DrawVertex), &Vertices->x);
	glTexCoordPointer(2, GL_FLOAT, sizeof(DrawVertex), &Vertices->u);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	glDrawArrays(GL_QUAD_STRIP, 0, SegmentCount*2+4);

	glDisable(GL_TEXTURE_2D);
	//glDisable(GL_BLEND);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);

	delete[] Vertices;

#else
	// Debug:
	for(C4RopeElement* cur = Front; cur != NULL; cur = cur->Next)
	{
		if(!cur->GetObject())
		{
			glBegin(GL_QUADS);
			glColor3f(1.0f, 0.0f, 0.0f);
			glVertex2f(fixtof(cur->x)-1.5f, fixtof(cur->y)-1.5f);
			glVertex2f(fixtof(cur->x)-1.5f, fixtof(cur->y)+1.5f);
			glVertex2f(fixtof(cur->x)+1.5f, fixtof(cur->y)+1.5f);
			glVertex2f(fixtof(cur->x)+1.5f, fixtof(cur->y)-1.5f);
			glEnd();
		}

		// Draw links
		for(C4RopeLink* link = cur->FirstLink; link != NULL; link = link->Next)
		{
			glBegin(GL_QUADS);
			glColor3f(1.0f, 1.0f, 1.0f);
			glVertex2f(fixtof(link->x)-1.0f, fixtof(link->y)-1.0f);
			glVertex2f(fixtof(link->x)-1.0f, fixtof(link->y)+1.0f);
			glVertex2f(fixtof(link->x)+1.0f, fixtof(link->y)+1.0f);
			glVertex2f(fixtof(link->x)+1.0f, fixtof(link->y)-1.0f);
			glEnd();
		}

		const float vx = fixtof(cur->GetVx());
		const float vy = fixtof(cur->GetVy());
		const float v = sqrt(vx*vx + vy*vy);
		if(v > 0.1)
		{
			glBegin(GL_TRIANGLES);
			glColor3f(BoundBy(v/2.5f, 0.0f, 1.0f), 0.0f, 1.0f);
			glVertex2f(fixtof(cur->GetX()) + vx/v*4.0f, fixtof(cur->GetY()) + vy/v*4.0f);
			glVertex2f(fixtof(cur->GetX()) - vy/v*1.5f, fixtof(cur->GetY()) + vx/v*1.5f);
			glVertex2f(fixtof(cur->GetX()) + vy/v*1.5f, fixtof(cur->GetY()) - vx/v*1.5f);
			glEnd();
		}

		const float fx = fixtof(cur->fcx);
		const float fy = fixtof(cur->fcy);
		const float f = sqrt(fx*fx + fy*fy);
		if(f > 0.1)
		{
			glBegin(GL_TRIANGLES);
			glColor3f(0.0f, 1.0f, BoundBy(v/2.5f, 0.0f, 1.0f));
			glVertex2f(fixtof(cur->GetX()) + fx/f*4.0f, fixtof(cur->GetY()) + fy/f*4.0f);
			glVertex2f(fixtof(cur->GetX()) - fy/f*1.5f, fixtof(cur->GetY()) + fx/f*1.5f);
			glVertex2f(fixtof(cur->GetX()) + fy/f*1.5f, fixtof(cur->GetY()) - fx/f*1.5f);
			glEnd();
		}

		const float rx = fixtof(cur->rx);
		const float ry = fixtof(cur->ry);
		const float r = sqrt(rx*rx + ry*ry);
		if(r > 0.1)
		{
			glBegin(GL_TRIANGLES);
			glColor3f(0.0f, BoundBy(r/2.5f, 0.0f, 1.0f), 1.0f);
			glVertex2f(fixtof(cur->x) + rx/r*4.0f, fixtof(cur->y) + ry/r*4.0f);
			glVertex2f(fixtof(cur->x) - ry/r*1.5f, fixtof(cur->y) + rx/r*1.5f);
			glVertex2f(fixtof(cur->x) + ry/r*1.5f, fixtof(cur->y) - rx/r*1.5f);
			glEnd();
		}
	}
#endif
}

C4RopeList::C4RopeList()
{
	for(unsigned int i = 0; i < Ropes.size(); ++i)
		delete Ropes[i];
}

C4Rope* C4RopeList::CreateRope(C4Object* first_obj, C4Object* second_obj, C4Real segment_length, C4DefGraphics* graphics)
{
	Ropes.push_back(new C4Rope(RopeAul.GetPropList(), first_obj, second_obj, segment_length, graphics));
	return Ropes.back();
}

void C4RopeList::RemoveRope(C4Rope* rope)
{
	for(std::vector<C4Rope*>::iterator iter = Ropes.begin(); iter != Ropes.end(); ++iter)
	{
		if(*iter == rope)
		{
			Ropes.erase(iter);
			delete rope;
			break;
		}
	}
}

void C4RopeList::Execute()
{
	// TODO: Note that Rope execution is completely independent from
	// each other -- if it turns out to be a bottleneck it could very
	// easily be run in parallel!
	for(unsigned int i = 0; i < Ropes.size(); ++i)
		Ropes[i]->Execute();
}

void C4RopeList::Draw(C4TargetFacet& cgo, C4BltTransform* pTransform)
{
	ZoomData z;
	pDraw->GetZoom(&z);

	glPushMatrix();
	ApplyZoomAndTransform(z.X, z.Y, z.Zoom, pTransform);
	glTranslatef(cgo.X-cgo.TargetX, cgo.Y-cgo.TargetY, 0.0f);

	for(unsigned int i = 0; i < Ropes.size(); ++i)
		Ropes[i]->Draw(cgo, pTransform);

	glPopMatrix();
}

void C4RopeList::ClearPointers(C4Object* obj)
{
	for(unsigned int i = 0; i < Ropes.size(); ++i)
		Ropes[i]->ClearPointers(obj);
}
