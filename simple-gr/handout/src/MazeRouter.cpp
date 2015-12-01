/*
 * MazeRouter.cpp
 * Work on this file to complete your maze router
 */

using namespace std;

#include "SimpleGR.h"

///////////////////////////////////////////////////////////////////////////////
// Implement an A* search based maze routing algorithm
// and a corresponding back-trace procedure
// The function needs to correctly deal with the following conditions:
// 1. Only search within a bounding box defined by botleft and topright points
// 2. Control if any overflow on the path is allowed or not
///////////////////////////////////////////////////////////////////////////////
CostType SimpleGR::routeMaze(Net& net, bool allowOverflow, const Point &botleft,
                             const Point &topright, const EdgeCost &func, vector<Edge*> &path)
{
  // find out the ID of the source and sink gcells
  const IdType srcGCellId = getGCellId(net.gCellOne);
  const IdType snkGCellId = getGCellId(net.gCellTwo);

  // insert the source gcell into the priority queue
  priorityQueue.setGCellCost(srcGCellId, 0., 0., NULLID);

  // Instantiate the Cost function to calculate the Manhattan distance between
  // two gcells. This distance is used as the heuristic cost for A* search.
  ManhattanCost &lb = ManhattanCost::getFunc();
//A*(const Point src, const Point snk)
{
GCell GCELL, snk; 
  // A* search kernel
  // Loop until all "frontiers" in the priority queue are exhausted, or when
  // the sink gcell is found.
  do{ 
    // YOUR A* search CODE GOES IN HERE
	GCELL = getGCell(priorityQueue.getBestGCell()); 
	snk = getGCell(snkGCellId);
	if(snkGCellId ==  getGCellId(GCELL)){
   		break;} //got to get out of the loop somehow
	priorityQueue.rmBestGCell(); 
	ADDQUEUE(GCELL,  priorityQueue, GCELL.incX, snk );
	ADDQUEUE(GCELL,  priorityQueue, GCELL.incY, snk );
	ADDQUEUE(GCELL,  priorityQueue, GCELL.incZ, snk );
	ADDQUEUE(GCELL,  priorityQueue, GCELL.decX, snk );
	ADDQUEUE(GCELL,  priorityQueue, GCELL.decY, snk );
	ADDQUEUE(GCELL,  priorityQueue, GCELL.decZ, snk );


    // YOUR A* search CODE ENDS HERE
  } while (!priorityQueue.isEmpty());
//current= node in openset having the lowest f_score[]

  // now backtrace and build up the path, if we found one
  // back-track from sink to source, and fill up 'path' vector with all the edges that are traversed
  if (priorityQueue.isGCellVsted(snkGCellId)) {    // YOUR backtrace CODE GOES IN HERE
//	path.push(EDGE 	);	
    // YOUR backtrace CODE ENDS HERE
  }
  // calculate the accumulated cost of the path
  const CostType finalCost = 
      priorityQueue.isGCellVsted(snkGCellId) ?
          priorityQueue.getGCellData(snkGCellId).pathCost : numeric_limits<CostType>::max();

  // clean up
  priorityQueue.clear();

  return finalCost;
}
