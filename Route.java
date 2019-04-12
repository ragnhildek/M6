import java.util.Vector;

public class Route {	
	public int number;
	public Vector<Node> path;
	public float profit;
	public float reducedCost;
	public Vehicle vehicle;
	public Vector<Integer> pickupNodesVisited;
}
