package roadgraph;

public class Edge {
	private String name;
	private String type;
	private double length;
	
	public Edge(String n, String t, double len) {
		name = n;
		type = t;
		length = len;
	}
	public String getName() {
		return new String(name);
	}
	
	public String getType() {
		return new String(type);
	}
	
	public double getLength() {
		return length;
	}
	
}
