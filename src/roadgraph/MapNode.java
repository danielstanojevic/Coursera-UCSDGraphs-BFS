package roadgraph;
//Daniel Stanojevic
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import geography.GeographicPoint;

public class MapNode implements Comparable<MapNode>{
	private GeographicPoint location;
	private List<MapEdge> edges;
	private List<MapNode> neighbors;
	private String name;
	private Double pathWeight;
	private MapNode previous;
	
	public List<MapEdge> getEdges(){
		return edges;
	}
	
	
	public List<MapNode> getNeighbors(){
		return this.neighbors;
	}
	
	public Double getEdgeWeight(GeographicPoint end){
		MapEdge target = new MapEdge();
		if (this.getEdges() == null){
			return null;
		}
		for (MapEdge edge : this.getEdges()){
			if ((this.getLocation() == edge.getStart()) && (end == edge.getEnd())){
				target = edge;
			}
		}
		return target.getDistance();
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
		this.pathWeight = Double.MAX_VALUE;
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

	/*
	 * @param pathWeight is the weight from the start node to this node
	 */
	public Double getPathWeight() {
		return pathWeight;
	}
	
	/*
	 * @param pathWeight is the weight from the start node to this node
	 */
	public void setPathWeight(Double pathWeight) {
		this.pathWeight = pathWeight;
	}

	/*
	 * @param previous is the previous node along the path from the start towards the goal
	 */
	public MapNode getPrevious() {
		return previous;
	}

	/*
	 * @param previous is the previous node along the path from the start towards the goal
	 */
	public void setPrevious(MapNode previous) {
		this.previous = previous;
	}

	@Override
	public int compareTo(MapNode node) {//compare location instead of pathweight?
		return this.getPathWeight().compareTo(node.getPathWeight());
	}

	public List<MapNode> getNeighborsFromEdges(HashMap <GeographicPoint, MapNode> vertices) {
		List<MapNode> neighbors = new ArrayList<>();
		if (this.edges == null){
			return neighbors;
		}
		for (MapEdge edge : this.getEdges()){
			if (this.getLocation() == edge.getStart()){
				neighbors.add(vertices.get(edge.getEnd()));
				//MapNode newNode = new MapNode(edge.getEnd());
				//newNode.setPathWeight(this.getPathWeight()+this.location.distance(newNode.location));
				//need to update pathweight in neighbors
				//for (MapNode element : neighbors){
					//if (element.location == newNode.location){
						//neighbors.get(neighbors.indexOf(element)).setPathWeight(newNode.getPathWeight());
					//}
				//}
				//neighbors.add(newNode);//this.get(edge.getEnd()));//new MapNode(edge.getEnd()));
			}
		}
		return neighbors;
	}
	
	@Override
	public String toString() {
		return "Location: "+location.toString()+" PathWeight: "+pathWeight;
	}
	
}
