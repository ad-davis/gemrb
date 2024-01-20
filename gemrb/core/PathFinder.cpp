/* GemRB - Engine Made with preRendered Background
 * Copyright (C) 2020 The GemRB Project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 *
 */

// This file implements the pathfinding logic for actors
// The main logic is in Map::FindPath, which is an
// implementation of the A* algorithm.
// GemRB uses two overlaid representation of the world: the searchmap and the navmap.
// Pathfinding is done on the searchmap and movement is done on the navmap.
// The navmap is bigger than the searchmap by a factor of (16, 12) on the (x, y) axes.
// Moving to each node in the path thus becomes an automatic regulation problem
// which is solved with a P regulator, see Scriptable.cpp

#include "FibonacciHeap.h"
#include "GameData.h"
#include "Map.h"
#include "PathFinder.h"
#include "RNG.h"
#include "Scriptable/Actor.h"

#include <array>
#include <limits>

namespace GemRB {

constexpr size_t DEGREES_OF_FREEDOM = 8;
constexpr size_t RAND_DEGREES_OF_FREEDOM = 16;
constexpr unsigned int SEARCHMAP_SQUARE_DIAGONAL = 20; // sqrt(16 * 16 + 12 * 12)
constexpr std::array<char, DEGREES_OF_FREEDOM> dxAdjacent{{ 1, 1, 1, 0, -1, -1, -1,  0}};
constexpr std::array<char, DEGREES_OF_FREEDOM> dyAdjacent{{-1, 0, 1, 1,  1,  0, -1, -1}};

// Cosines
constexpr std::array<double, RAND_DEGREES_OF_FREEDOM> dxRand{{0.000, -0.383, -0.707, -0.924, -1.000, -0.924, -0.707, -0.383, 0.000, 0.383, 0.707, 0.924, 1.000, 0.924, 0.707, 0.383}};
// Sines
constexpr std::array<double, RAND_DEGREES_OF_FREEDOM> dyRand{{1.000, 0.924, 0.707, 0.383, 0.000, -0.383, -0.707, -0.924, -1.000, -0.924, -0.707, -0.383, 0.000, 0.383, 0.707, 0.924}};

static inline void freePath(PathListNode* start) {
	PathListNode *thisNode = start;
	while (thisNode) {
		PathListNode *nextNode = thisNode->Next;
		delete thisNode;
		thisNode = nextNode;
	}
}

// Find the best path of limited length that brings us the farthest from d
PathList Map::RunAway(const Point &s, const Point &d, unsigned int size, int maxPathLength, bool backAway, const Actor *caller) const
{
	PathList result;
	if (!caller || !caller->GetSpeed()) return result;
	Point p = s;
	double dx = s.x - d.x;
	double dy = s.y - d.y;
	char xSign = 1, ySign = 1;
	size_t tries = 0;
	NormalizeDeltas(dx, dy, double(gamedata->GetStepTime()) / caller->GetSpeed());
	while (SquaredDistance(p, s) < unsigned(maxPathLength * maxPathLength * SEARCHMAP_SQUARE_DIAGONAL * SEARCHMAP_SQUARE_DIAGONAL)) {
		Point rad(std::lround(p.x + 3 * xSign * dx), std::lround(p.y + 3 * ySign * dy));
		if (!(GetBlockedInRadius(rad, size) & PathMapFlags::PASSABLE)) {
			tries++;
			// Give up and call the pathfinder if backed into a corner
			// should we return nullptr instead, so we don't accidentally get closer to d?
			// it matches more closely the iwd beetles in ar1015, but is too restrictive — then they can't move at all
			if (tries > RAND_DEGREES_OF_FREEDOM) break;
			// Random rotation
			xSign = RandomFlip() ? -1 : 1;
			ySign = RandomFlip() ? -1 : 1;
			continue;
		}
		p = rad;
	}
	int flags = PF_SIGHT;
	if (backAway) flags |= PF_BACKAWAY;
	result = FindPath(s, p, size, size, flags, caller);
	return result;
}

PathListNode *Map::RandomWalk(const Point &s, int size, int radius, const Actor *caller) const
{
	if (!caller || !caller->GetSpeed()) return nullptr;
	NavmapPoint p = s;
	size_t i = RAND<size_t>(0, RAND_DEGREES_OF_FREEDOM - 1);
	double dx = 3 * dxRand[i];
	double dy = 3 * dyRand[i];

	NormalizeDeltas(dx, dy, double(gamedata->GetStepTime()) / caller->GetSpeed());
	size_t tries = 0;
	while (SquaredDistance(p, s) < unsigned(radius * radius * SEARCHMAP_SQUARE_DIAGONAL * SEARCHMAP_SQUARE_DIAGONAL)) {
		if (!(GetBlockedInRadius(p + Point(dx, dy), size) & PathMapFlags::PASSABLE)) {
			tries++;
			// Give up if backed into a corner
			if (tries > RAND_DEGREES_OF_FREEDOM) {
				return nullptr;
			}
			// Random rotation
			i = RAND<size_t>(0, RAND_DEGREES_OF_FREEDOM - 1);
			dx = 3 * dxRand[i];
			dy = 3 * dyRand[i];
			NormalizeDeltas(dx, dy, double(gamedata->GetStepTime()) / caller->GetSpeed());
			p = s;
		} else {
			p.x += dx;
			p.y += dy;
		}
	}
	while (!(GetBlockedInRadius(p + Point(dx, dy), size) & (PathMapFlags::PASSABLE|PathMapFlags::ACTOR))) {
		p.x -= dx;
		p.y -= dy;
	}
	PathListNode *step = new PathListNode;
	const Size& mapSize = PropsSize();
	step->point = Clamp(p, Point(1, 1), Point((mapSize.w - 1) * 16, (mapSize.h - 1) * 12));
	step->orient = GetOrient(p, s);
	return step;
}

bool Map::TargetUnreachable(const Point &s, const Point &d, unsigned int size, bool actorsAreBlocking) const
{
	int flags = PF_SIGHT;
	if (actorsAreBlocking) flags |= PF_ACTORS_ARE_BLOCKING;
	PathList path = FindPath(s, d, size, 0, flags);
	bool foundPath = path.node != nullptr;
	if (foundPath) {
		freePath(path.node);
	}
	return !foundPath || !path.fullPath;
}

// Use this function when you target something by a straight line projectile (like a lightning bolt, arrow, etc)
PathListNode *Map::GetLine(const Point &start, const Point &dest, int flags) const
{
	orient_t Orientation = GetOrient(start, dest);
	return GetLine(start, dest, 1, Orientation, flags);
}

PathListNode *Map::GetLine(const Point &start, int Steps, orient_t Orientation, int flags) const
{
	Point dest = start;

	double xoff, yoff, mult;
	if (Orientation <= 4) {
		xoff = -Orientation / 4.0;
	} else if (Orientation <= 12) {
		xoff = -1.0 + (Orientation - 4) / 4.0;
	} else {
		xoff = 1.0 - (Orientation - 12) / 4.0;
	}

	if (Orientation <= 8) {
		yoff = 1.0 - Orientation / 4.0;
	} else {
		yoff = -1.0 + (Orientation - 8) / 4.0;
	}

	mult = 1.0 / std::max(std::fabs(xoff), std::fabs(yoff));

	dest.x += Steps * mult * xoff + 0.5;
	dest.y += Steps * mult * yoff + 0.5;

	return GetLine(start, dest, 2, Orientation, flags);
}

PathListNode *Map::GetLine(const Point &start, const Point &dest, int Speed, orient_t Orientation, int flags) const
{
	PathListNode *StartNode = new PathListNode;
	PathListNode *Return = StartNode;
	StartNode->point = start;
	StartNode->orient = Orientation;

	int Count = 0;
	int Max = Distance(start, dest);
	for (int Steps = 0; Steps < Max; Steps++) {
		Point p;
		p.x = start.x + ((dest.x - start.x) * Steps / Max);
		p.y = start.y + ((dest.y - start.y) * Steps / Max);

		//the path ends here as it would go off the screen, causing problems
		//maybe there is a better way, but i needed a quick hack to fix
		//the crash in projectiles
		if (p.x < 0 || p.y < 0) {
			return Return;
		}
		
		const Size& mapSize = PropsSize();
		if (p.x > mapSize.w * 16 || p.y > mapSize.h * 12) {
			return Return;
		}

		if (!Count) {
			StartNode->Next = new PathListNode;
			StartNode->Next->Parent = StartNode;
			StartNode = StartNode->Next;
			Count = Speed;
		} else {
			Count--;
		}

		StartNode->point = p;
		StartNode->orient = Orientation;
		bool wall = bool(GetBlocked(p) & (PathMapFlags::DOOR_IMPASSABLE | PathMapFlags::SIDEWALL));
		if (wall) switch (flags) {
			case GL_REBOUND:
				Orientation = ReflectOrientation(Orientation);
				// TODO: recalculate dest (mirror it)
				break;
			case GL_PASS:
				break;
			default: //premature end
				return Return;
		}
	}

	return Return;
}

Path Map::GetLinePath(const Point &start, const Point &dest, int Speed, orient_t Orientation, int flags) const
{
	int Count = 0;
	int Max = Distance(start, dest);
	Path path;
	path.reserve(Max);
	path.push_back(PathNode {start, Orientation});
	auto StartNode = path.begin();
	for (int Steps = 0; Steps < Max; Steps++) {
		Point p;
		p.x = start.x + ((dest.x - start.x) * Steps / Max);
		p.y = start.y + ((dest.y - start.y) * Steps / Max);

		//the path ends here as it would go off the screen, causing problems
		//maybe there is a better way, but i needed a quick hack to fix
		//the crash in projectiles
		if (p.x < 0 || p.y < 0) {
			return path;
		}
		
		const Size& mapSize = PropsSize();
		if (p.x > mapSize.w * 16 || p.y > mapSize.h * 12) {
			return path;
		}

		if (!Count) {
			StartNode = path.insert(path.end(), {p, Orientation});
			Count = Speed;
		} else {
			Count--;
			StartNode->point = p;
			StartNode->orient = Orientation;
		}

		bool wall = bool(GetBlocked(p) & (PathMapFlags::DOOR_IMPASSABLE | PathMapFlags::SIDEWALL));
		if (wall) switch (flags) {
			case GL_REBOUND:
				Orientation = ReflectOrientation(Orientation);
				// TODO: recalculate dest (mirror it)
				break;
			case GL_PASS:
				break;
			default: //premature end
				return path;
		}
	}

	return path;
}

PathListNode* Map::GetLine(const Point &p, int steps, orient_t orient) const
{
	PathListNode *step = new PathListNode;
	step->point.x = p.x + steps * SEARCHMAP_SQUARE_DIAGONAL * dxRand[orient];
	step->point.y = p.y + steps * SEARCHMAP_SQUARE_DIAGONAL * dyRand[orient];
	const Size& mapSize = PropsSize();
	step->point = Clamp(step->point, Point(1, 1), Point((mapSize.w - 1) * 16, (mapSize.h - 1) * 12));
	step->orient = GetOrient(step->point, p);
	return step;
}

PathList Map::FindBestPath(const Point &s, const Point &d, unsigned int size, const Actor *caller, unsigned int minDistance) const {
	// we first find a simple path without caring about actors (non-bumpables still are considered)
	// we then use that as a basis for giving up if the path with actors gets too long
	PathList firstPath = FindPath(s, d, size, minDistance, PF_SIGHT, caller);
	// if we can't find a path when actors don't block, there is no chance when they do
	if (firstPath.node && firstPath.fullPath) {
		PathList finalPath;
		unsigned int distanceLimit = 0;
		if (firstPath.maxDistance) distanceLimit = firstPath.maxDistance*(2+100/firstPath.maxDistance); // if is just to guard against division by zero
		finalPath = FindPath(s, d, size, minDistance, PF_SIGHT | PF_ACTORS_ARE_BLOCKING, caller, distanceLimit);
		if (!finalPath.fullPath && caller && caller->ValidTarget(GA_CAN_BUMP)) {
			// TODO: at this point, it might be worth attempting to recalculate the path with a slightly different starting point
			if (core->InDebugMode(ID_PATHFINDER)) Log(DEBUG, "FindBestPath", "{} re-pathing ignoring actors", fmt::WideToChar{caller->GetShortName()});
			freePath(finalPath.node);
			finalPath = firstPath;
		} else {
			freePath(firstPath.node);
			if (!finalPath.fullPath) {
				if (core->InDebugMode(ID_PATHFINDER)) Log(DEBUG, "FindBestPath", "{} re-pathing for closest possible", fmt::WideToChar{caller->GetShortName()});
				// for now, taking this option off the table
				//freePath(finalPath.node);
				//finalPath.node = nullptr;
			}
		}
		return finalPath;
	} else {
		return firstPath;
	}
}

// Find a path from start to goal, ending at the specified distance from the
// target (the goal must be in sight of the end, if PF_SIGHT is specified)
PathList Map::FindPath(const Point &s, const Point &d, unsigned int size, unsigned int minDistance, int flags, const Actor *caller, unsigned int maxDistance) const
{
	PathList result;

	bool actorsAreBlocking = flags & PF_ACTORS_ARE_BLOCKING;
	if (core->InDebugMode(ID_PATHFINDER)) Log(DEBUG, "FindPath", "s = {}, d = {}, caller = {}, size = {}, minDist = {}, maxDist = {}, actorsBlock = {}", s, d, caller ? MBStringFromString(caller->GetShortName()) : "nullptr", size, minDistance, maxDistance, actorsAreBlocking);
	
	// TODO: we could optimize this function further by doing everything in SearchmapPoint and converting at the end
	NavmapPoint nmptDest = d;
	NavmapPoint nmptSource = s;
	if (!(GetBlockedInRadius(d, size) & PathMapFlags::PASSABLE)) {
		// If the desired target is blocked, find the path
		// to the nearest reachable point.
		// Also avoid bumping a still actor out of its position,
		// but stop just before it
		AdjustPositionNavmap(nmptDest);
	}
	
	if (nmptDest == nmptSource) return result;
	
	SearchmapPoint smptSource = Map::ConvertCoordToTile(nmptSource);
	SearchmapPoint smptDest = Map::ConvertCoordToTile(nmptDest);
	
	const Size& mapSize = PropsSize();
	if (!mapSize.PointInside(smptSource)) return result;

	// Initialize data structures
	FibonacciHeap<PQNode> open;
	std::vector<NavmapPoint> parents(mapSize.Area(), Point(0, 0));
	std::vector<unsigned short> distFromStart(mapSize.Area(), std::numeric_limits<unsigned short>::max());
	std::vector<unsigned int> distances(mapSize.Area(), 0);
	distFromStart[smptSource.y * mapSize.w + smptSource.x] = 0;
	parents[smptSource.y * mapSize.w + smptSource.x] = nmptSource;
	open.emplace(PQNode(nmptSource, 0));
	bool foundPath = false;
	unsigned int squaredMinDist = minDistance * minDistance;
	unsigned int closestDist = std::numeric_limits<unsigned int>::max();
	unsigned int closestDistFromStart = 0;
	NavmapPoint nmptClosest;

	while (!open.empty()) {
		NavmapPoint nmptCurrent = open.top().point;
		open.pop();
		SearchmapPoint smptCurrent = Map::ConvertCoordToTile(nmptCurrent);
		if (parents[smptCurrent.y * mapSize.w + smptCurrent.x].IsZero()) {
			continue;
		}

		if (smptCurrent == smptDest) {
			nmptDest = nmptCurrent;
			foundPath = true;
			break;
		} else if (parents[smptCurrent.y * mapSize.w + smptCurrent.x] != nmptCurrent) {
			auto sqrDist = SquaredDistance(nmptCurrent, nmptDest);
			if (minDistance && sqrDist < squaredMinDist) {
				if (!(flags & PF_SIGHT) || IsVisibleLOS(nmptCurrent, d)) {
					if (core->InDebugMode(ID_PATHFINDER)) Log(DEBUG, "FindPath", "{} settled for minimum distance", caller ? MBStringFromString(caller->GetShortName()) : "nullptr");
					smptDest = smptCurrent;
					nmptDest = nmptCurrent;
					foundPath = true;
					break;
				}
			} else if (sqrDist < closestDist) {
				closestDist = sqrDist;
				closestDistFromStart = SquaredDistance(nmptSource, nmptCurrent);
				nmptClosest = nmptCurrent;
			}
		}

		for (size_t i = 0; i < DEGREES_OF_FREEDOM; i++) {
			NavmapPoint nmptChild(nmptCurrent.x + 16 * dxAdjacent[i], nmptCurrent.y + 12 * dyAdjacent[i]);
			SearchmapPoint smptChild = Map::ConvertCoordToTile(nmptChild);
			// Outside map
			if (smptChild.x < 0 ||	smptChild.y < 0 || smptChild.x >= mapSize.w || smptChild.y >= mapSize.h) continue;
			if (actorsAreBlocking) {
				// If there's an actor, check it can be bumped away
				const Actor* childActor = GetActor(nmptChild, GA_NO_DEAD | GA_NO_UNSCHEDULED);
				bool childIsUnbumpable = childActor && childActor != caller && (actorsAreBlocking || !childActor->ValidTarget(GA_ONLY_BUMPABLE));
				if (childIsUnbumpable) continue;
			}

			PathMapFlags childBlockStatus = GetBlockedInRadius(nmptChild, size);
			bool childBlocked = !(childBlockStatus & (PathMapFlags::PASSABLE | (actorsAreBlocking ? PathMapFlags::UNMARKED : PathMapFlags::ACTOR) | PathMapFlags::TRAVEL));
			if (childBlocked) continue;

			// Weighted heuristic. Finds sub-optimal paths but should be quite a bit faster
			const float HEURISTIC_WEIGHT = 1.5;
			SearchmapPoint smptCurrent2 = Map::ConvertCoordToTile(nmptCurrent);
			unsigned short oldDist = distFromStart[smptChild.y * mapSize.w + smptChild.x];

			// if the new distance will be shorter, check if it is reachable and if so do the update
			unsigned short newDist = distFromStart[smptCurrent2.y * mapSize.w + smptCurrent2.x] + Distance(smptCurrent2, smptChild);
			if (newDist < oldDist && IsWalkableTo(nmptCurrent, nmptChild, actorsAreBlocking, caller)) {
				parents[smptChild.y * mapSize.w + smptChild.x] = nmptCurrent;
				distFromStart[smptChild.y * mapSize.w + smptChild.x] = newDist;
			}

			if (distFromStart[smptChild.y * mapSize.w + smptChild.x] < oldDist) {
				// Calculate heuristic
				int xDist = smptChild.x - smptDest.x;
				int yDist = smptChild.y - smptDest.y;
				// Tie-breaking used to smooth out the path
				int dxCross = smptDest.x - smptSource.x;
				int dyCross = smptDest.y - smptSource.y;
				int crossProduct = std::abs(xDist * dyCross - yDist * dxCross) >> 3;
				double distance = std::hypot(xDist, yDist);
				double heuristic = HEURISTIC_WEIGHT * (distance + crossProduct);
				double estDist = distFromStart[smptChild.y * mapSize.w + smptChild.x] + heuristic;
				distances[smptChild.y * mapSize.w + smptChild.x] = std::round(estDist);
				if (!maxDistance || estDist <= maxDistance) {
					PQNode newNode(nmptChild, estDist);
					open.emplace(newNode);
				}
			}
		}
	}

	NavmapPoint nmptCurrent;
	if (foundPath) {
		nmptCurrent = nmptDest;
	} else if (!nmptClosest.IsZero() && closestDistFromStart > 1600) {
		nmptCurrent = nmptClosest;
	} else {
		if (core->InDebugMode(ID_PATHFINDER)) {
			if (caller) {
				Log(DEBUG, "FindPath", "Pathing failed for {}", fmt::WideToChar{caller->GetShortName()});
			} else {
				Log(DEBUG, "FindPath", "Pathing failed");
			}
		}
		return result;
	}
	// we build the path backwards here
	PathListNode *resultPath = nullptr;
	SearchmapPoint smptCurrent = Map::ConvertCoordToTile(nmptCurrent);
	NavmapPoint nmptNext = parents[smptCurrent.y * mapSize.w + smptCurrent.x];
	auto greatestDist = distances[smptCurrent.y * mapSize.w + smptCurrent.x];
	while (!resultPath || nmptCurrent != nmptNext) {
		// this is independent of piecing together the path, it's just convenient to calculate the greatest distance in this loop
		auto dist = distances[smptCurrent.y * mapSize.w + smptCurrent.x];
		if (dist > greatestDist) greatestDist = dist;

		SearchmapPoint smptNext = Map::ConvertCoordToTile(nmptNext);

		// smooth out the path by skipping next if we have line of sight to the node after the next
		NavmapPoint nmptNext2 = parents[smptNext.y * mapSize.w + smptNext.x];
		// this check always sets actorsAreBlocking to true regardless of flags because otherwise it tends to construct paths
		// that get stuck on walls. not sure exactly why
		if (nmptNext != nmptNext2 && IsWalkableTo(nmptNext2, nmptCurrent, true, caller)) {
			nmptNext = nmptNext2;
			continue;
		}

		PathListNode *newStep = new PathListNode;
		newStep->point = nmptCurrent;
		newStep->Next = resultPath;
		// movement in general allows characters to walk backwards given that
		// the destination is behind the character (within a threshold), and
		// that the distance isn't too far away
		// we approximate that with a relaxed collinearity check and intentionally
		// skip the first step, otherwise it doesn't help with iwd beetles in ar1015
		if (flags & PF_BACKAWAY && resultPath && std::abs(area2(nmptCurrent, resultPath->point, nmptNext)) < 300) {
			newStep->orient = GetOrient(nmptNext, nmptCurrent);
		} else {
			newStep->orient = GetOrient(nmptCurrent, nmptNext);
		}
		if (resultPath) {
			resultPath->Parent = newStep;
		}
		resultPath = newStep;

		nmptCurrent = nmptNext;
		smptCurrent = smptNext;
		nmptNext = nmptNext2;
	}
	result.node = resultPath;
	result.fullPath = foundPath;
	result.maxDistance = greatestDist;

	return result;
}

void Map::NormalizeDeltas(double &dx, double &dy, double factor)
{
	constexpr double STEP_RADIUS = 2.0;

	double ySign = std::copysign(1.0, dy);
	double xSign = std::copysign(1.0, dx);
	dx = std::fabs(dx);
	dy = std::fabs(dy);
	double dxOrig = dx;
	double dyOrig = dy;
	if (dx == 0.0) {
		dy = STEP_RADIUS;
	} else if (dy == 0.0) {
		dx = STEP_RADIUS;
	} else {
		double q = STEP_RADIUS / std::sqrt(dx*dx+dy*dy);
		dx = dx * q;
		dy = dy * q * 0.75f;
	}
	dx = std::min(dx * factor, dxOrig);
	dy = std::min(dy * factor, dyOrig);
	dx = std::ceil(dx) * xSign;
	dy = std::ceil(dy) * ySign;
}

}
