package roadgraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import geography.GeographicPoint;

public class GraphNode {
	private HashMap<GeographicPoint, Edge> neighbors;
	
	public GraphNode(){
		neighbors = new HashMap<GeographicPoint, Edge>();
	}
	public void addNeighbor(GeographicPoint p, Edge e) {
		neighbors.put(p, e);
	}
	
	public List<GeographicPoint> getNeighbors() {
		return new ArrayList<GeographicPoint>(neighbors.keySet());
	}
	
	public HashMap<GeographicPoint, Edge> getEdges() {
		return new HashMap<GeographicPoint, Edge>(neighbors);
	}
	public boolean existNeighbor(GeographicPoint n) {
		if (neighbors.containsKey(n)) {
			return true;
		}
		return false;
	}
}
