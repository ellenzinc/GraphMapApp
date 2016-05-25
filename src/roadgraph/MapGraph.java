/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import com.sun.xml.internal.bind.v2.runtime.unmarshaller.XsiNilLoader.Array;
import com.sun.xml.internal.ws.addressing.v200408.ProblemAction;

import geography.GeographicPoint;
import sun.java2d.loops.GraphicsPrimitiveProxy;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private int numEdges;
	private int numVertices;
	private HashMap<GeographicPoint, GraphNode> nodes;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		numEdges = 0;
		numVertices = 0;
		nodes = new HashMap<GeographicPoint, GraphNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return null;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		if (nodes.containsKey(location)) {
			return false;
		}
		GraphNode newNode = new GraphNode();
		nodes.put(location, newNode);
		numVertices ++;
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

		//TODO: Implement this method in WEEK 2
		// Check if the points already exist
		if (from == null || to == null || !(nodes.containsKey(from)) || !(nodes.containsKey(to)) ||
				 roadName == null || roadType == null || length < 0 ) {
			throw new IllegalArgumentException();
		}
		
		// Create an Edge and add neighbors to the nodes
		Edge e = new Edge(roadName, roadType, length);
		nodes.get(from).addNeighbor(to, e);
		numEdges ++;
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
		// TODO: Implement this method in WEEK 2
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		if (start.getX() == goal.getX() && start.getY() == goal.getY()) {
			List<GeographicPoint> path = new ArrayList<GeographicPoint>();
			path.add(start);
			return path;
		}
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		Queue<GeographicPoint> q = new LinkedList<GeographicPoint>();
		q.add(start);
		nodeSearched.accept(start);
		while (!q.isEmpty()) {
			GeographicPoint current = q.poll();
			List<GeographicPoint>neighbors = nodes.get(current).getNeighbors();
			for (GeographicPoint g: neighbors) {
				if (parentMap.containsKey(g)){
					continue;
				}
				q.add(g);
				nodeSearched.accept(g);
				parentMap.put(g, current);
				if (g.getX() == goal.getX() && g.getY() == goal.getY()) {
					return constructPath(start, goal, parentMap);
				}
			}
		}		
		return null;
	}
	
	
	/**
	 * Construct a path when start != goal
	 * @param start
	 * @param goal
	 * @param parentMap
	 * @return
	 */
	private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal, 
			HashMap<GeographicPoint, GeographicPoint> parentMap) {
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		GeographicPoint current = goal;
		while(current.getX() != start.getX() || current.getY() != start.getY()) {
			path.add(current);
			current = parentMap.get(current);
		}
		path.add(start);
		Collections.reverse(path);
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
		// TODO: Implement this method in WEEK 3
		// special case when we are already at the goal;

		// Initialization
		HashMap<GeographicPoint, DistanceNode> nodeToDist = new 
				HashMap<GeographicPoint, DistanceNode> ();
		
		PriorityQueue<DistanceNode> pq = 
				new PriorityQueue<DistanceNode>(getNumVertices(), new ReverseComparator());
		for (GeographicPoint p: nodes.keySet()) {
			nodeToDist.put(p, new DijkstrasDistance(start, p));
		}
		pq.add(nodeToDist.get(start));
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		while(!pq.isEmpty()) {
			printPriorityQueue(pq);
			DistanceNode current = pq.poll();

			GeographicPoint currentPos = current.getPosition();
			nodeSearched.accept(currentPos);
			if (!visited.contains(currentPos)) {
				visited.add(currentPos);
				//nodeSearched.accept(current.getPosition());
				GraphNode edges = nodes.get(currentPos);
				HashMap<GeographicPoint, Edge> neighbors = edges.getEdges();
				Iterator<?> it = neighbors.entrySet().iterator();
				if (currentPos.getX() == goal.getX() && currentPos.getY() == currentPos.getY()) {
					return constructPath(start, goal, parentMap);
				}				
				while(it.hasNext()) {
					@SuppressWarnings("unchecked")
					Map.Entry<GeographicPoint, Edge> next = 
							(Map.Entry<GeographicPoint, Edge>)it.next();
					GeographicPoint neighbor = next.getKey();
					if (visited.contains(neighbor)){
						continue;
					}
					double newDistance = next.getValue().getLength() + 
							nodeToDist.get(currentPos).getDistance();
					if (nodeToDist.get(neighbor).getDistance() > newDistance) {
						parentMap.put(neighbor, currentPos);
						nodeToDist.get(neighbor).setDistance(newDistance);
						pq.add(nodeToDist.get(neighbor));
					}
				}
			}	
		}
		

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}
	
	private void printPriorityQueue(PriorityQueue<DistanceNode> pq) {
		PriorityQueue<DistanceNode> copyPq = new PriorityQueue<DistanceNode> (pq);
		while(!copyPq.isEmpty()) {
			DistanceNode current = copyPq.poll();
			System.out.println(current.getPosition() + ": "+current.getDistance());
		}
		System.out.println("");
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
		// TODO: Implement this method in WEEK 3
		HashMap<GeographicPoint, AstarDistance> nodeToDist = new 
				HashMap<GeographicPoint, AstarDistance> ();
		
		PriorityQueue<DistanceNode> pq = 
				new PriorityQueue<DistanceNode>(getNumVertices(), new ReverseComparator());
		
		for (GeographicPoint p: nodes.keySet()) {
			nodeToDist.put(p, new AstarDistance(start, p, goal));
		}
		pq.add(nodeToDist.get(start));
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		while(!pq.isEmpty()) {
			printPriorityQueue(pq);
			DistanceNode current = pq.poll();

			GeographicPoint currentPos = current.getPosition();
			nodeSearched.accept(currentPos);
			if (!visited.contains(currentPos)) {
				visited.add(currentPos);
				//nodeSearched.accept(current.getPosition());
				GraphNode edges = nodes.get(currentPos);
				HashMap<GeographicPoint, Edge> neighbors = edges.getEdges();
				Iterator<?> it = neighbors.entrySet().iterator();
				if (currentPos.getX() == goal.getX() && currentPos.getY() == currentPos.getY()) {
					return constructPath(start, goal, parentMap);
				}				
				while(it.hasNext()) {
					@SuppressWarnings("unchecked")
					Map.Entry<GeographicPoint, Edge> next = 
							(Map.Entry<GeographicPoint, Edge>)it.next();
					GeographicPoint neighbor = next.getKey();
					if (visited.contains(neighbor)){
						continue;
					}
					double newDistance = next.getValue().getLength() + 
							nodeToDist.get(currentPos).getDistanceFromSource() + nodeToDist.get(neighbor).getDistanceToDest();
					if (nodeToDist.get(neighbor).getDistance() > newDistance) {
						parentMap.put(neighbor, currentPos);
						nodeToDist.get(neighbor).setDistance(newDistance);
						pq.add(nodeToDist.get(neighbor));
					}
				}
			}	
		}		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/**
	 * Print Graph
	 */
	public void printGraph() {
		// used for testing
		Iterator it = nodes.entrySet().iterator();
		while (it.hasNext()) {
			Map.Entry<GeographicPoint, GraphNode> pair = (Map.Entry<GeographicPoint, GraphNode>) it.next();
			System.out.println(pair.getKey()+":");
			GraphNode gNode = pair.getValue();
			HashMap<GeographicPoint, Edge> edges = gNode.getEdges();
			Iterator it2 = edges.entrySet().iterator();
			while (it2.hasNext()) {
				Map.Entry<GeographicPoint, Edge> edge = 
						(Map.Entry<GeographicPoint, Edge>) it2.next();
				System.out.println("   " + edge.getKey() + ": " + edge.getValue().getName() 
						+ ", "+ edge.getValue().getType() + ", " + edge.getValue().getLength());
			}
		}
	}
	
	/**
	 * Test bfs
	 * @param args
	 */
	public void testBFS(GeographicPoint p1, GeographicPoint p2) {
		List<GeographicPoint> path = bfs(p1, p2);
		System.out.println("The path between Point "+p1 + " and Point "+ p2 + " is: ");
		if (path == null) {
			System.out.println("Unknown");
		} else {
			System.out.println(Arrays.toString(path.toArray()));
		}
	}
	
	
	public void testDijkstras(GeographicPoint p1, GeographicPoint p2) {
		List<GeographicPoint> path = dijkstra(p1, p2);
		System.out.println("The path between Point "+p1 + " and Point "+ p2 + " is: ");
		if (path == null) {
			System.out.println("Unknown");
		} else {
			System.out.println(Arrays.toString(path.toArray()));
		}
		
	}
	
	
	public void testAstar(GeographicPoint p1, GeographicPoint p2) {
		List<GeographicPoint> path = aStarSearch(p1, p2);
		System.out.println("The path between Point "+p1 + " and Point "+ p2 + " is: ");
		if (path == null) {
			System.out.println("Unknown");
		} else {
			System.out.println(Arrays.toString(path.toArray()));
		}
		
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		System.out.println("Print the loaded graph:");
		theMap.printGraph();
		
		//GeographicPoint point1 = new GeographicPoint(4.0, 1.0);
		//GeographicPoint point2 = new GeographicPoint(4.0, 1.0);
		
		//theMap.testBFS(point1, point2);
		GeographicPoint point1 = new GeographicPoint(1.0, 1.0);
		GeographicPoint point2 = new GeographicPoint(8.0, -1.0);
		
		theMap.testAstar(point1, point2);
		
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
