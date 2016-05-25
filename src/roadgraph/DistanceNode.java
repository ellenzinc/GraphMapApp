package roadgraph;

import geography.GeographicPoint;

public abstract class DistanceNode {
	protected double distFromSource;
	protected GeographicPoint position;
	//protected double distToDest; // this is the estimated distance to the destination	
	

	//public DistanceNode(double dfs, GeographicPoint pos, double dtd) {
		//distFromSource = dfs;
		//position = pos;
		//distToDest = dtd;
	//}
	
	public abstract double getDistance();
	
	public double getDistanceFromSource() {
		return distFromSource;
	}
	
	public GeographicPoint getPosition() {
		return new GeographicPoint(position.getX(), position.getY());
	}
	
	public void setDistance(double distance) {
		distFromSource = distance;
	}
}
