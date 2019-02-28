/**
 * @author UCSD MOOC development team and Daniel Stanojevic
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODONE: Add your member variables here in WEEK 3
	private int numVertices;
	private int numEdges;
	private HashMap <GeographicPoint, MapNode> vertices;

	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODONE: Implement in this constructor in WEEK 3
		numVertices = 0;
		numEdges = 0;
		vertices = new HashMap<GeographicPoint, MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODONE: Implement this method in WEEK 3
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODONE: Implement this method in WEEK 3
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODONE: Implement this method in WEEK 3
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph and returns false.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if (location == null || vertices.containsKey(location)){
			return false;
		}
		numVertices++;
		MapNode newNode = new MapNode(location);
		vertices.put(location, newNode);
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		MapNode fromNode = vertices.get(from);
		
		if (vertices.containsKey(from) && 
				vertices.containsKey(to) && 
				from != null && 
				to != null && 
				length > 0 && 
				fromNode.getEdges() != null) {
					numEdges++;
					MapEdge newEdge = new MapEdge(from, to, roadName, roadType, length);
					fromNode.addEdge(newEdge);
					fromNode.addNeighbor(vertices.get(to));
		}
		else {
			System.out.println("Vertices contains Key from: "+vertices.containsKey(from));
			System.out.println("Vertices contains Key to: "+vertices.containsKey(to));
			System.out.println("From: "+from.toString());
			System.out.println("To: "+to.toString());
			System.out.println("Length: "+length);
			System.out.println("From distance to > 0? "+from.distance(to));
			System.out.println("fromNode.getEdges not null? "+fromNode.getEdges()!=null);
			
			
			throw new IllegalArgumentException();
		}
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		toExplore.add(vertices.get(start));

		MapNode curr = null;
		if (start == null || goal == null){
			System.out.println("start and goal are required, but not found");
			return null;
		}
		while(!toExplore.isEmpty()){
			curr = toExplore.remove();
			nodeSearched.accept(curr.getLocation());//hook for visualization
			
			if (curr.equals(vertices.get(goal))){
				System.out.println("Found it!");
				break;	
			}
			for (MapNode neighbor : curr.getNeighbors()){
				if (!visited.contains(neighbor)){
					visited.add(neighbor);
					parentMap.put(neighbor, curr);
					toExplore.add(neighbor);
				}
			}
		}	
		// If we get here then there's no path
		if(!curr.equals(vertices.get(goal))){
			System.out.println("No path found");
			return null;
		}
		
		path = constructPath(start, goal, parentMap);
		
		return path;
	}
	
	/** Construct the path from start to goal using the parentMap
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap The HashMap of child nodes and parent nodes
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal,
			HashMap<MapNode, MapNode> parentMap) {
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = vertices.get(goal);

		while(!current.equals(vertices.get(start))){
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}
		
		path.addFirst(start);
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		//Comparator for pathWeight field in remaining priority queue
		Comparator<MapNode> pathWeightSorter = Comparator.comparing(MapNode::getPathWeight);
		//Initialize variables
		PriorityQueue<MapNode> remaining = new PriorityQueue<>( pathWeightSorter );		
		HashSet<MapNode> visited = new HashSet<>();
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		boolean found = false;
		
		//initialize all path weights to infinity
		for (MapNode node : vertices.values()){
			node.setPathWeight(Double.POSITIVE_INFINITY);
		}
		
		//initialize start weight to zero and add to queue
		vertices.get(start).setPathWeight(0.0);
		remaining.add(vertices.get(start));

		while(!remaining.isEmpty()){
			MapNode curr = remaining.poll();
			if (!visited.contains(curr)){
				visited.add(curr);
				// Hook for visualization.  See writeup.
				nodeSearched.accept(curr.getLocation());
				System.out.println("DIJKSTRA visiting "+curr.toString());
				//If we found the goal return the path
				if (curr.getLocation().equals(goal)){
					found = true;
					break;
				}
				//check edge distances to each neighbor of current node for a shorter path
				for (MapNode neighbor : curr.getNeighborsFromEdges(vertices)){
					Double currToNeighborDistance = curr.getLocation().distance(neighbor.getLocation());
					Double costSoFar = curr.getPathWeight() + currToNeighborDistance;
					Double heuristicDistance = 0.0;
					//set shorter path if found and add neighbor to queue
					if ((neighbor.getPathWeight() > costSoFar+heuristicDistance)){
						neighbor.setPathWeight(costSoFar+heuristicDistance);
						neighbor.setPrevious(curr);
						remaining.add(neighbor);
					}
				}
			}
		}
		if(!found){
			System.out.println("Dijkstra path not found.");
			return null;
		}
		System.out.println("Visited: "+visited.size());
		path = constructPath(start, goal, "Dijkstra");
		return path;
	}
	/** Helper method to construct a path from start to goal
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal, String searchType) {
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = vertices.get(goal);
		
		while(current != null && !current.equals(vertices.get(start))){
			path.addFirst(current.getLocation());
			current = current.getPrevious();
		}
		path.addFirst(start);
		System.out.println(searchType+" path:");
		System.out.println(path.toString());
		return path;
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		//Comparator for pathWeight field in remaining priority queue
		Comparator<MapNode> pathWeightSorter = Comparator.comparing(MapNode::getPathWeight);
		//Initialize variables
		PriorityQueue<MapNode> remaining = new PriorityQueue<>( pathWeightSorter );		
		HashSet<MapNode> visited = new HashSet<>();
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		boolean found = false;
		
		//initialize all path weights to infinity
		for (MapNode node : vertices.values()){
			node.setPathWeight(Double.POSITIVE_INFINITY);
		}
		
		//initialize start weight to zero and add to queue
		vertices.get(start).setPathWeight(0.0);
		remaining.add(vertices.get(start));

		while(!remaining.isEmpty()){
			MapNode curr = remaining.poll();
			if (!visited.contains(curr)){
				visited.add(curr);
				// Hook for visualization.  See writeup.
				nodeSearched.accept(curr.getLocation());
				System.out.println("A* visiting "+curr.toString());
				//If we found the goal return the path
				if (curr.getLocation().equals(goal)){
					found = true;
					break;
				}
				//check edge distances to each neighbor of current node for a shorter path
				for (MapNode neighbor : curr.getNeighborsFromEdges(vertices)){
					Double currToNeighborDistance = curr.getLocation().distance(neighbor.getLocation());
					Double costSoFar = curr.getPathWeight() + currToNeighborDistance;
					Double heuristicDistance = neighbor.getLocation().distance(goal)-curr.getLocation().distance(goal);
					//set shorter path if found and add neighbor to queue
					if ((neighbor.getPathWeight() > costSoFar+heuristicDistance)){
						neighbor.setPathWeight(costSoFar+heuristicDistance);
						neighbor.setPrevious(curr);
						remaining.add(neighbor);
					}
				}
			}
		}
		if(!found){
			System.out.println("A* path not found.");
			return null;
		}
		System.out.println("Visited: "+visited.size());
		path = constructPath(start, goal, "A*");
		return path;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		
		 MapGraph simpleTestMap = new MapGraph();
			GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
			
			GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
			GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
			
			System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
			List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		  
			// Reinitialize map to make sure it's in the correct state
			simpleTestMap = new MapGraph();
			GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
			
			List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
			
			
			MapGraph testMap = new MapGraph();
			GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
			
			// A very simple test using real data
			testStart = new GeographicPoint(32.869423, -117.220917);
			testEnd = new GeographicPoint(32.869255, -117.216927);
			System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
			testroute = testMap.dijkstra(testStart,testEnd);
			testroute2 = testMap.aStarSearch(testStart,testEnd);
			
			
			// A slightly more complex test using real data
			testStart = new GeographicPoint(32.8674388, -117.2190213);
			testEnd = new GeographicPoint(32.8697828, -117.2244506);
			System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
			testroute = testMap.dijkstra(testStart,testEnd);
			testroute2 = testMap.aStarSearch(testStart,testEnd);
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}
	
}
