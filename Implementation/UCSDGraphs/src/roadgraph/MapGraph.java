/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.Consumer;

import geography.GeographicPoint;
import sun.misc.Queue;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */
public class MapGraph {
	// TODO: Add your member variables here in WEEK 2
	private HashMap<GeographicPoint, MapNode> pointNodes;
	private HashSet<MapEdge> edges;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		// TODO: Implement in this constructor in WEEK 2
		pointNodes = new HashMap<GeographicPoint, MapNode>();
		edges = new HashSet<MapEdge>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		// TODO: Implement this method in WEEK 2
		return pointNodes.values().size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		// TODO: Implement this method in WEEK 2
		return pointNodes.keySet();
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		// TODO: Implement this method in WEEK 2
		return edges.size();
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location
	 *            The location of the intersection
	 * @return true if a node was added, false if it was not (the node was
	 *         already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		// TODO: Implement this method in WEEK 2
		MapNode node = pointNodes.get(location);
		if (node == null) {
			node = new MapNode(location);
			pointNodes.put(location, node);
			return true;
		}
		System.out.println("The vertex already present in the graph, " + location.toString());
		return false;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from
	 *            The starting point of the edge
	 * @param to
	 *            The ending point of the edge
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 * @param length
	 *            The length of the road, in km
	 * @throws IllegalArgumentException
	 *             If the points have not already been added as nodes to the
	 *             graph, if any of the arguments is null, or if the length is
	 *             less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {

		// TODO: Implement this method in WEEK 2
		MapNode startNode = pointNodes.get(from);
		MapNode endNode = pointNodes.get(to);
		if (startNode.equals(null) || endNode.equals(null))
			throw new IllegalArgumentException("Vertex does not exist in graph");
		MapEdge mapEdge = new MapEdge(roadType, roadName, startNode, endNode, length);
		edges.add(mapEdge);
		startNode.addEdge(mapEdge);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 2
		/* It has running time of O(|E| + |V|) */
		if (!nullCheckTrue(start, goal)) {
			System.out.println("It is this null");
			return null;
		}
		MapNode startNode = pointNodes.get(start);
		MapNode endNode = pointNodes.get(goal);

		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		Queue<MapNode> toSearchNext = new Queue<MapNode>();
		HashSet<MapNode> visitedNodes = new HashSet<MapNode>();

		toSearchNext.enqueue(startNode);
		MapNode currentNode = null;

		while (!toSearchNext.isEmpty()) {
			try {
				currentNode = toSearchNext.dequeue();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			nodeSearched.accept(currentNode.getLocation()); // Hook for
			// visualization.
			// See writeup.
			if (currentNode.equals(endNode))
				break;
			for (MapNode neighbor : getNeighbors(currentNode)) {
				if (!visitedNodes.contains(neighbor)) {
					visitedNodes.add(neighbor);
					parentMap.put(neighbor, currentNode);
					toSearchNext.enqueue(neighbor);
				}
			}

		}

		return constructPath(parentMap, startNode, endNode, currentNode.equals(endNode));
	}

	private Set<MapNode> getNeighbors(MapNode currentNode) {
		return currentNode.getNeighbors();
	}

	private boolean nullCheckTrue(GeographicPoint start, GeographicPoint goal) {
		if (start == null || goal == null)
			throw new NullPointerException("Can't find path from null node/s");
		if (pointNodes.get(start) == null) {
			System.err.println("Start node " + start + " doesn't exist in graph");
			return false;
		}
		if (pointNodes.get(goal) == null) {
			System.err.println("Goal node " + goal + " doesn't exist in graph");
			return false;
		}
		return true;
	}

	private List<GeographicPoint> constructPath(HashMap<MapNode, MapNode> pMap, MapNode sNode, MapNode eNode,
			boolean found) {
		if (!found) {
			System.out.println("Path not found between " + sNode.getLocation() + " and " + eNode.getLocation());
			return null;
		}
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = eNode;

		while (!current.equals(sNode)) {
			path.addFirst(current.getLocation());
			current = pMap.get(current);
		}

		// add start
		path.addFirst(sNode.getLocation());
		return path;
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 3
		/* It has running time of O(|E|*log|E| + |V|) */
		return searchOnWeightedGraph(start, goal, nodeSearched, (neighbor, endNnode) -> 0.0);
	}

	/* Week3 Solution */
	private HashMap<MapNode, Double> getAllDistancesForMapNode(MapNode currentNode) {
		HashMap<MapNode, Double> distanceForMapNode = new HashMap<>();
		for (MapEdge mapEdge : currentNode.getEdges()) {
			distanceForMapNode.put(mapEdge.getEndNode(), mapEdge.getLength());
		}
		return distanceForMapNode;
	}

	private void initializeDistanceToInfinity() {
		for (MapNode mapNode : pointNodes.values()) {
			mapNode.setDistance(Double.MAX_VALUE);
			mapNode.setHeuristicDistance(Double.MAX_VALUE);
		}
	}
	/* end of Week3 Solution */

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 3
		return searchOnWeightedGraph(start, goal, nodeSearched,
				(neighbor, endNode) -> neighbor.getLocation().distance(endNode.getLocation()));
	}

	private List<GeographicPoint> searchOnWeightedGraph(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched, BiFunction<MapNode, MapNode, Double> f) {
		if (!nullCheckTrue(start, goal)) {
			System.out.println("It is this null");
			return null;
		}
		MapNode startNode = pointNodes.get(start);
		MapNode endNode = pointNodes.get(goal);

		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		PriorityQueue<MapNode> toSearchNext = new PriorityQueue<MapNode>();
		HashSet<MapNode> visitedNodes = new HashSet<MapNode>();

		initializeDistanceToInfinity();
		MapNode currentNode = null;

		startNode.setDistance(0.0);
		startNode.setHeuristicDistance(0.0);

		toSearchNext.add(startNode);

		while (!toSearchNext.isEmpty()) {
			currentNode = toSearchNext.poll();

			if (!visitedNodes.contains(currentNode)) {
				visitedNodes.add(currentNode);
				nodeSearched.accept(currentNode.getLocation()); // Hook for
				// visualization.
				// See writeup.
				if (currentNode.equals(endNode)) {
					break;
				}
				HashMap<MapNode, Double> distancesMap = getAllDistancesForMapNode(currentNode);
				for (MapNode neighbor : getNeighbors(currentNode)) {
					if (!visitedNodes.contains(neighbor)) {
						double dist = currentNode.getDistance() + distancesMap.get(neighbor);
						if (dist < neighbor.getDistance()) {
							/*
							 * Here when algorithm is Dijkstra f.apply(neighbor,
							 * endNode) will add zero making f(n) as only the
							 * distance g(n) between vertex and start node f(n)
							 * = g(n)
							 * 
							 * When algorithm is A* f.apply(neighbor, endNode)
							 * will add heuristic distance h(n) from vertex to
							 * endNode along with g(n) f(n) = g(n) + h(n)
							 */
							neighbor.setDistance(dist + f.apply(neighbor, endNode));
							parentMap.put(neighbor, currentNode);
							toSearchNext.offer(neighbor);
						}
					}
				}
			}
		}
		System.out.println("Visited: " + visitedNodes.size());
		return constructPath(parentMap, startNode, endNode, currentNode.equals(endNode));

	}

	public static void main(String[] args) {
		System.out.print("Making a new map...");
		MapGraph theMap1 = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap1);
		System.out.println("DONE.");

		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start, end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start, end);
	}

}
