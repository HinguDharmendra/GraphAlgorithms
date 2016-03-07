package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

public class MapNode implements Comparable<Object> {

	private GeographicPoint location;
	private HashSet<MapEdge> edges;

	/* Week3 Solution */
	private double distance;
	private double heuristicDistance;

	double getDistance() {
		return distance;
	}

	void setDistance(double distance) {
		this.distance = distance;
	}

	double getHeuristicDistance() {
		return heuristicDistance;
	}

	void setHeuristicDistance(double heuristicDistance) {
		this.heuristicDistance = heuristicDistance;
	}
	/* end of Week3 Solution */

	MapNode(GeographicPoint location) {
		this.location = location;
		edges = new HashSet<MapEdge>();

		/* Week 3 Solution */
		this.distance = 0.0;
		this.heuristicDistance = 0.0;
		/* end of Week 3 Solution */

	}

	GeographicPoint getLocation() {
		return location;
	}

	Set<MapEdge> getEdges() {
		return edges;
	}

	public boolean equals(Object o) {
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode) o;
		return node.location.equals(this.location);
	}

	int HashCode() {
		return location.hashCode();
	}

	public String toString() {
		String node = "--> Node at location " + location + " has internsections at, ";
		for (MapEdge mapEdge : edges) {
			node += mapEdge.getRoadName() + ", ";
		}
		return node;
	}

	Set<MapNode> getNeighbors() {
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for (MapEdge mapEdge : edges) {
			neighbors.add(mapEdge.getOtherNodeEdge(this));
		}
		return neighbors;
	}

	void addEdge(MapEdge mapEdge) {
		edges.add(mapEdge);
	}

	/* Week 3 Solution */
	@Override
	public int compareTo(Object o) {
		MapNode m = (MapNode) o;
		return ((Double) this.getDistance()).compareTo((Double) m.getDistance());
	}
	/* end of Week 3 Solution */
}
