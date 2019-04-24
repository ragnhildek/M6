
import java.util.ArrayList;
import java.util.Vector;


	public class Label {
		//public int bestLabelNumber;
		public Vehicle vehicle;
		public Vector<Node> path;
		public Vector<Integer> pickupNodesVisited;
		public double pickupDual;
		public double totalPickupDual;
		public double vehicleDual;
		public double reducedCost;
		public double time; 
		public double profit;
		public double weightCapacityUsed;
		public double volumeCapacityUsed;
		public Label predesessor;
		public Node node;
//		public Vector<Integer> path;
		public Vector<Integer> unreachablePickupNodes;
		public double dailyDrivingTime;
		public double consecutiveDrivingTime;
		public double startTimeDailyRest;
		public double startTimeIntermediateBreak;
		public Vector<Integer> openNodes; //pickupnodes
		public int numberDailyRests;
		public double consecutiveWorkingTime;
	//	public double waitingTime;
		public double totalDistance;
		

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
