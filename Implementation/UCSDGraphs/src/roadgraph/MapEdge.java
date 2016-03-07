package roadgraph;

public class MapEdge {

	private MapNode startNode;
	private MapNode endNode;
	private double length;
	private String roadName;
	private String roadType;

	MapEdge(String roadType, String roadName, MapNode startNode, MapNode endNode, double distance) {
		this.startNode = startNode;
		this.endNode = endNode;
		this.length = distance;
		this.roadType = roadType;
		this.roadName = roadName;

	}

	MapEdge(String roadType, MapNode startNode, MapNode endNode) {
		this(roadType, "", startNode, endNode, 0);
	}

	MapEdge(String roadType, String roadName, MapNode startNode, MapNode endNode) {
		this(roadType, roadName, startNode, endNode, 0);
	}

	MapNode getStartNode() {
		return startNode;
	}

	MapNode getEndNode() {
		return endNode;
	}

	double getLength() {
		return length;
	}

	String getRoadName() {
		return roadName;
	}

	String getRoadType() {
		return roadType;
	}

	public String toString() {
		return "--> Edge between \t" + startNode.getLocation() + " and " + endNode.getLocation() + " has a distance of "
				+ length + " and known as " + roadType + " on " + roadName;
	}

	MapNode getOtherNodeEdge(MapNode node) {
		if (node.equals(startNode)) {
			return endNode;
		} else if (node.equals(endNode)) {
			return startNode;
		}
		throw new IllegalArgumentException("No such point exist in a node");

	}
}
