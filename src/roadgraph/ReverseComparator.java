package roadgraph;

import java.util.Comparator;

public class ReverseComparator implements Comparator<DistanceNode> {

	@Override
	public int compare(DistanceNode o1, DistanceNode o2) {
		Double dist1 = o1.getDistance();
		Double dist2 = o2.getDistance();
		return dist1.compareTo(dist2);
	}
}
