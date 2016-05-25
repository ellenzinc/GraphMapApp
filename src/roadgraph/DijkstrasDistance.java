package roadgraph;

import geography.GeographicPoint;

public class DijkstrasDistance extends DistanceNode {

	public DijkstrasDistance(GeographicPoint start, GeographicPoint goal) {
		// TODO Auto-generated constructor stub
		if (start.getX() == goal.getX() && start.getY() == goal.getY()){
			distFromSource = 0;
		} else {
			distFromSource = Double.MAX_VALUE;			
		}
		position = goal;
	}

	@Override
	public double getDistance() {
		return distFromSource;
	}
	
	

}
