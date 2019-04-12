
import java.util.ArrayList;
import java.util.Vector;


	public class Label {
		//public int bestLabelNumber;
		public Vehicle vehicle;
		public Vector<Node> path;
		public Vector<Integer> pickupNodesVisited;
		public float pickupDual;
		public float totalPickupDual;
		public float vehicleDual;
		public float reducedCost;
		public float time; 
		public float profit;
		public float weightCapacityUsed;
		public float volumeCapacityUsed;
		public Label predesessor;
		public Node node;
//		public Vector<Integer> path;
		public Vector<Integer> unreachablePickupNodes;
		public float dailyDrivingTime;
		public float consecutiveDrivingTime;
		public float startTimeDailyRest;
		public float startTimeIntermediateBreak;
		public Vector<Integer> openNodes; //pickupnodes
		public int numberDailyRests;
		public float consecutiveWorkingTime;
	//	public float waitingTime;
		public float totalDistance;
		

	public String toString() {
		String string = "Node: " + node.number + ", Location: " + node.location + " , " + node.locationName + ", Time: " + time + ", Profit: "+ profit + ", Reduced cost: " + reducedCost +
				", unreachablePickupNodes: " + unreachablePickupNodes + ", openNodes: " + openNodes + ", dailyDrivingTime: " + dailyDrivingTime + ", startTimeDailyRest: " + startTimeDailyRest + ", numberDailyRests: " + numberDailyRests + ", StartTimeIntermediateBreak: "+ startTimeIntermediateBreak + ", consecutiveDrivingTime: " + consecutiveDrivingTime + ", workingTime: " + consecutiveWorkingTime 
				+ ", totalDistance: " + totalDistance+ ", WeightCapacityUsed: " + weightCapacityUsed + ", VolumeCapacityUsed: " + volumeCapacityUsed;
		
		//if(!path.isEmpty()) {
		//for (Node  i : path) { 
		//	string += i.number;
		//}}
		Label temp = predesessor;
//		
		while(temp!=null) {
			string+=", Predessesor: "+temp.node.number;
			temp=temp.predesessor;
		}
//		
//		System.out.println(string);
//		System.out.println("");
//		System.out.println("");
		return string;
	}
}
