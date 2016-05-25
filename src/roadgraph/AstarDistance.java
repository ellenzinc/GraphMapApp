package roadgraph;

import geography.GeographicPoint;

public class AstarDistance extends DistanceNode {
	private double distToDest;
	public AstarDistance(GeographicPoint start, GeographicPoint current, GeographicPoint goal) {
		if (current.getX() == start.getX() && current.getY() == start.getY()){
			distFromSource = 0.0;
		} else {
			distFromSource = Double.MAX_VALUE;	
		}
		distToDest = current.distance(goal);
		position = current;
	}

	@Override
	public double getDistance() {
		// TODO Auto-generated method stub
		if (distFromSource == Double.MAX_VALUE)
			return distFromSource;
		return distFromSource + distToDest;
	}

	public void setDistToDest(double distance) {
		distToDest =  distance;
	}
	
	public double getDistanceToDest() {
		return distToDest;
	}
}
