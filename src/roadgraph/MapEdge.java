package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	private GeographicPoint start;
	private GeographicPoint end;
	private String roadName;
	private String roadType;
	private double distance;
	
	public MapEdge(){
		this.start = new GeographicPoint(0,0);
		this.end = new GeographicPoint(0,0);
		this.roadName = null;
		this.roadType = null;
		this.distance = 0.0;
	}
	
	public MapEdge(GeographicPoint start, GeographicPoint end, String roadName, String roadType){
		this.start = start;
		this.end = end;
		this.roadName = roadName;
		this.roadType = roadType;
		this.distance = start.distance(end);
	}
	
	public MapEdge(GeographicPoint start, GeographicPoint end, String roadName, String roadType, double distance){
		this.start = start;
		this.end = end;
		this.roadName = roadName;
		this.roadType = roadType;
		this.distance = distance;
	}
	
	public MapEdge(GeographicPoint start, GeographicPoint end){
		this.start = start;
		this.end = end;
		this.roadName = null;
		this.roadType = null;
		this.distance = start.distance(end);
	}

	public GeographicPoint getStart() {
		return start;
	}

	public GeographicPoint getEnd() {
		return end;
	}

	public String getRoadName() {
		return roadName;
	}


	public String getRoadType() {
		return roadType;
	}


	public double getDistance() {
		return distance;
	}



}
