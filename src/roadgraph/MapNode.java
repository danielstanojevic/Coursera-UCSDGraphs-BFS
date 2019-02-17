package roadgraph;

import java.util.ArrayList;
import java.util.List;
import geography.GeographicPoint;

public class MapNode {
	private GeographicPoint location;
	private List<MapEdge> edges;
	private List<MapNode> neighbors;
	private String name;
	
	public List<MapEdge> getEdges(){
		return edges;
	}
	
	public List<MapNode> getNeighbors(){
		return this.neighbors;
	}
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph and returns false.
	 * @param neighbor  The neighbor node that we are adding to the current node.
	 */
	public void addNeighbor(MapNode neighbor){
		if (neighbor != null){
			this.neighbors.add(neighbor);
		}	
	}
	
	public void addEdge(MapEdge e){
		this.edges.add(e);
	}
	
	public void addEdge(GeographicPoint start, GeographicPoint end){
		if (!this.edges.isEmpty()) {
			this.edges.add(new MapEdge(start,end));
		}
	}
	
	public MapNode(GeographicPoint location) {
		this.location = location;
		this.name = "";
		this.edges = new ArrayList<MapEdge>();
		this.neighbors = new ArrayList<MapNode>();
	}
	
	public MapNode(GeographicPoint location, String name) {
		this.location = location;
		this.name = name;
		this.edges = new ArrayList<MapEdge>();
		this.neighbors = new ArrayList<MapNode>();
	}
	
	public MapNode(GeographicPoint location, String name, List<MapEdge> edges){
		this.location = location;
		this.name = name;
		this.edges = edges;
		this.neighbors = new ArrayList<MapNode>();
	}

	public GeographicPoint getLocation() {
		return location;
	}

	public String getName() {
		return name;
	}
	
}
