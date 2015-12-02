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
GCell GCELL, snk=getGCell(snkGCellId);
{
  // A* search kernel
  // Loop until all "frontiers" in the priority queue are exhausted, or when
  // the sink gcell is found.
  do{ 
    // YOUR A* search CODE GOES IN HERE
    //start off with the best GCELL to help decide where to go next
	GCELL = getGCell(priorityQueue.getBestGCell()); 
	//here we check to see if we have reaced our end result, if so we get out of the loop.
	if(snkGCellId ==  getGCellId(GCELL)){
   		break;} //got to get out of the loop somehow
  	 //we just used our bestGCell and it wasn't there so we move it to closed
	priorityQueue.rmBestGCell(); 
	// get the edge Id to get the next GCELL values to check for
	int checkValues=0; 
	IdType EDGEID, LOOP[6];
	if(GCELL.x < topright.x-1){
	 LOOP[checkValues] = GCELL.incX;
	checkValues++;}
	if(GCELL.x > botleft.x-1) 
	{ LOOP[checkValues] = GCELL.decX;
	checkValues++;} 
	if(GCELL.Y < topright.Y-1){
	 LOOP[checkValues] = GCELL.incY;
	checkValues++;}
	if(GCELL.Y > botleft.Y-1) 
	{ LOOP[checkValues] = GCELL.decY;
	checkValues++;} 
	//make sure it exisits
	for(int i=0; i < checkValues; i++)
	{
		EDGEID=LOOP[i];
		if(EDGEID != NULLID )
		{
			//puts GCELL1 and GCELL2 in their own GCELL to make cooding easier 
		GCell GCELL1 = *grEdgeArr[EDGEID].gcell1;
		GCell GCELL2 = *grEdgeArr[EDGEID].gcell2; 
		//gets the path cost of the current GCELL to add to our calculations 
		float pathCost = priorityQueue.getGCellData(getGCellId(GCELL)).pathCost;
			//figures out which GCELL to use (the one that is not the same as the current)
			if(getGCellId(GCELL1) == getGCellId(GCELL))
			{
				//checks to see if the GCELL Has been visited or is in the queue, if not it adds it 
				 if(!priorityQueue.isGCellVsted(getGCellId(GCELL2)))
				{
				// gives it the ID, path cost, heuristic cost, and manhattan cost
					priorityQueue.setGCellCost(getGCellId(GCELL2), pathCost+func(EDGEID)+lb(snk,GCELL2) ,pathCost+func(EDGEID), getGCellId(GCELL));  
				}//checks to see if there is a better cost for that cell, if so it adds it
				else if( priorityQueue.getGCellData(getGCellId(GCELL2)).totalCost > pathCost+func(EDGEID)+lb(snk,GCELL2))
				{
					priorityQueue.setGCellCost(getGCellId(GCELL2), pathCost+func(EDGEID)+lb(snk,GCELL2) ,pathCost+func(EDGEID), getGCellId(GCELL)); 
				}
			}
		
			else
			{
			 	if(!priorityQueue.isGCellVsted(getGCellId(GCELL1)))
				{
				// gives it the ID, path cost, heuristic cost, and manhattan cost
					priorityQueue.setGCellCost(getGCellId(GCELL1), pathCost+func(EDGEID)+lb(snk,GCELL1) ,pathCost+func(EDGEID), getGCellId(GCELL));  
				}//checks to see if there is a better cost for that cell, if so it adds it
				else if( priorityQueue.getGCellData(getGCellId(GCELL)).totalCost > pathCost+func(EDGEID)+lb(snk,GCELL2))
				{
					priorityQueue.setGCellCost(getGCellId(GCELL1), pathCost+func(EDGEID)+lb(snk,GCELL1) ,pathCost+func(EDGEID), getGCellId(GCELL)); 
				}
			}
	
		}
	}
	// just the same as above for the next 5 sides of the GCELL, I tried doing a function but kept getting errors and it seemed longer
	//to make the function work than to just copy and change the code for the value (only difference is the EDGEID)
		 
	
    // YOUR A* search CODE ENDS HERE
  } while (!priorityQueue.isEmpty());

GCELL=getGCell(snkGCellId);
IdType test, EDGEID; 
  // now backtrace and build up the path, if we found one
  // back-track from sink to source, and fill up 'path' vector with all the edges that are traversed
  if (priorityQueue.isGCellVsted(snkGCellId)) {    // YOUR backtrace CODE GOES IN
  // creates a loop to go through all the previous cells, stops when the source is the parent
	while(priorityQueue.getGCellData(getGCellId(GCELL)).parentGCell != srcGCellId)
	//stores the edge so it can be pushed onto the vector
	test = priorityQueue.getGCellData(getGCellId(GCELL)).parentGCell;  
	EDGEID=IDEDGE(GCELL, getGCell(test));
	Edge backtrace= grEdgeArr[EDGEID];
    	path.push_back(&backtrace);
    	//navigates to the next GCELL to continue its back trace
    	GCELL = getGCell(priorityQueue.getGCellData(getGCellId(GCELL)).parentGCell);
  	cout<< getGCellId(GCELL)<<endl;
	  // YOUR backtrace CODE ENDS HERE
  }
	//gets and stores the final edge  
  	test = priorityQueue.getGCellData(getGCellId(GCELL)).parentGCell;  
	EDGEID=IDEDGE(GCELL, getGCell(test));
	Edge backtrace=grEdgeArr[EDGEID];
    	path.push_back(&backtrace);
  // calculate the accumulated cost of the path (outputs the past via snkGCELLID for total cost )
  const CostType finalCost = priorityQueue.getGCellData(snkGCellId).totalCost;
      priorityQueue.isGCellVsted(snkGCellId) ?
          priorityQueue.getGCellData(snkGCellId).pathCost : numeric_limits<CostType>::max();

  // clean up
  priorityQueue.clear();

  return finalCost;
}}
