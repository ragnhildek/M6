import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Vector;

public class Preprocessing {
	
//	public Vector<Node> nodes;
	public Vector<Node> pickupNodes;
	public Vector<Node> deliveryNodes;
	//public Vector<Node> nodesWithoutDepot;
	public Vector<Node> depot;
	public InstanceData inputdata;
	public ArrayList<float[]> unreachableNodesFromNode;
	public ArrayList<float[]> unreachableDelNodesFromNode;
	public Hashtable<String, Float> unreachableDelPairs;
	public Vector<Vehicle> vehicles;
	public Vector<Node> nodes;
	// public static ArrayList<Hashtable<Integer, Boolean>> feasibilityTest; 
	
	public Preprocessing(Vector<Node> pickupNodes, Vector<Node> deliveryNodes, Vector<Vehicle> vehicles, InstanceData inputdata, Vector<Node> nodesWithoutDepot, Vehicle vehicle) {
		this.pickupNodes = pickupNodes;
	//	this.nodes = nodes;
		this.deliveryNodes = deliveryNodes;
		this.depot = depot;
		this.inputdata = inputdata;
	//	this.nodesWithoutDepot = nodesWithoutDepot;
		nodes = vehicle.nodes;
	//	for(int i=0; i < vehicle.nodes.size(); i++) {
	//		System.out.println(vehicle.nodes.get(i).location);
	//	}
	}
	
	
	public void unreachableNodeCombination() {
	//	for(Vehicle k : vehicles) {
		this.unreachableNodesFromNode = new ArrayList<float[]>();
		for(int i = 0; i < nodes.size();i++) {
			unreachableNodesFromNode.add(new float[nodes.size()]);
		}
	//}
		
		
		for(Node pickup : pickupNodes) {
			for(Node pickup2 : pickupNodes) {
				if(pickup!=pickup2) {
				//	System.out.println(pickup.location);
				//	System.out.println(pickup2.location);
//					System.out.println("checking for node "+pickup.number+" and "+pickup2.number);
					Node delivery2 = nodes.get(pickup2.number+1);
					Node delivery = nodes.get(pickup.number+1);
			//		System.out.println(delivery.location);
				//	System.out.println(delivery2.location);
					float time = pickup2.lateTimeWindow-(pickup.weight*inputdata.timeTonService)-
							inputdata.getTime(pickup, pickup2); // Checking if the time for traveling from pickup 1 to another pickup 2 is so large that the time window in pickup 2 will be violated
					float time2= delivery2.lateTimeWindow-pickup2.weight *inputdata.timeTonService-inputdata.getTime(pickup2, delivery2)-
							-pickup.weight *inputdata.timeTonService-inputdata.getTime(pickup, pickup2); // Checking whether when going from pickup 1 to pickup 2, and then to delivery 2 will violate the TW in delivery 2 
					float time3 = delivery.lateTimeWindow-delivery2.weight*inputdata.timeTonService -inputdata.getTime(delivery2, delivery)-
							pickup2.weight *inputdata.timeTonService-inputdata.getTime(pickup2, delivery2)-
							-pickup.weight *inputdata.timeTonService-inputdata.getTime(pickup, pickup2); // Checking whether going from pickup 1 to pickup 2, then to delivery 2, and at last to delivery 1 will violate the TW in delivery 1
					float time4 = delivery2.lateTimeWindow-delivery.weight*inputdata.timeTonService -inputdata.getTime(delivery, delivery2)-
							pickup2.weight *inputdata.timeTonService-inputdata.getTime(pickup2, delivery)-
							-pickup.weight *inputdata.timeTonService-inputdata.getTime(pickup, pickup2); // Checking whether going from pickup 1 to pickup 2, from pickup 2 to delivery 1, and then from delivery 1 to delivery 2 will violate the TW in delivery 2
					float time5 = delivery2.lateTimeWindow-pickup.weight*inputdata.timeTonService -inputdata.getTime(pickup2, delivery2)-
							delivery.weight *inputdata.timeTonService-inputdata.getTime(delivery, pickup2)-
							-pickup.weight *inputdata.timeTonService-inputdata.getTime(pickup, delivery); // Checking whether going from pickup 1 to delivery 1, from delivery 1 to pickup 2 and from pickup 2 to delivery 2 will violate the TW in delivery 2
					unreachableNodesFromNode.get(pickup.number)[pickup2.number] = Math.min(time,Math.min(time2, Math.max(time3, Math.max(time4, time5))));
//					System.out.println(pickup.earlyTimeWindow);
//					System.out.println((pickup.weight*inputdata.timeTonService));
//					System.out.println(inputdata.getTime(pickup, pickup2));
//					System.out.println("time: "+time+" TWL "+pickup2.lateTimeWindow);
//					if(Math.min(time,time2)<pickup.earlyTimeWindow) {
////						System.out.println("node "+pickup.number+" and "+pickup2.number);
////						System.out.println(Math.min(time,time2)+" < "+pickup.earlyTimeWindow);
////						unreachableNodesFromNode.get(pickup.number).add(pickup2.number);
////						continue;
//					}
//					time+=pickup2.weight *inputdata.timeTonService+inputdata.getTime(pickup2, delivery2);
//					if(time>delivery2.lateTimeWindow) {
//						System.out.println("node "+pickup.number+" and "+pickup2.number);
//						unreachableNodesFromNode.get(pickup.number).add(pickup2.number);
//						continue;
//					}
					
					
					time = pickup2.lateTimeWindow-(pickup.weight*inputdata.timeTonService)-
							inputdata.getTime(delivery, pickup2);
					time2= delivery2.lateTimeWindow-pickup2.weight *inputdata.timeTonService-inputdata.getTime(pickup2, delivery2)-
							 -pickup.weight *inputdata.timeTonService-inputdata.getTime(delivery, pickup2);
					unreachableNodesFromNode.get(delivery.number)[pickup2.number] = Math.min(time,time2);
//					if(Math.min(time,time2)<delivery.earlyTimeWindow) {
////						System.out.println("node "+pickup.number+" and "+pickup2.number);
////						System.out.println(Math.min(time,time2)+" < "+delivery.earlyTimeWindow);
////						unreachableNodesFromNode.get(pickup.number).add(pickup2.number);
////						continue;
//					}
//					time = delivery.earlyTimeWindow+pickup.weight *inputdata.timeTonService+
//							inputdata.getTime(delivery, pickup2);
//					if(time>pickup2.lateTimeWindow) {
//						System.out.println("node "+delivery.number+" and "+pickup2.number);
//						unreachableNodesFromNode.get(delivery.number).add(pickup2.number);
//						continue;
//					}
//					time+=pickup2.weight *inputdata.timeTonService+inputdata.getTime(pickup2, delivery2);
//					if(time>nodes.get(pickup2.number+1).lateTimeWindow) {
//						System.out.println("node "+delivery.number+" and "+pickup2.number);
//						unreachableNodesFromNode.get(delivery.number).add(pickup2.number);
//						continue;
//					}		
//							
				}
			}
			
		}
		
		
		
	}
	
	
	public void unreachableDeliveryNode() {
	//	for(Vehicle k : vehicles) {
		this.unreachableDelNodesFromNode = new ArrayList<float[]>();
		for(int i = 0; i < nodes.size();i++) {
			unreachableDelNodesFromNode.add(new float[nodes.size()]);
	//	}
	}
		
		
		for(Node delivery : deliveryNodes) {
			for(Node delivery2 : deliveryNodes) {
				if(delivery==delivery2) {
					unreachableDelNodesFromNode.get(delivery.number)[delivery2.number] = 1000;
				}
				else {
					Node pickup = nodes.get(delivery.number-1);
//					System.out.println("checking for node "+pickup.number+" and "+pickup2.number);
//					Node delivery2 = nodes.get(pickup2.number+1);
					float time = delivery2.lateTimeWindow-(pickup.weight*inputdata.timeTonService)-
							inputdata.getTime(delivery, delivery2); // Checking if going from delivery 1 to delivery 2 violates TW in delivery 2
//					float time2= delivery2.lateTimeWindow-pickup2.weight *inputdata.timeTonService+inputdata.getTime(pickup2, delivery2)-
//							Math.max(time, pickup2.earlyTimeWindow);
					unreachableDelNodesFromNode.get(delivery.number)[delivery2.number] = time;
//					System.out.println(pickup.earlyTimeWindow);
//					System.out.println((pickup.weight*inputdata.timeTonService));
//					System.out.println(inputdata.getTime(pickup, pickup2));
//					System.out.println("time: "+time+" TWL "+pickup2.lateTimeWindow);
					if(time<delivery.lateTimeWindow) {
						//System.out.println("node "+delivery.number+" and "+delivery2.number);
						//System.out.println(time+" < "+delivery.lateTimeWindow);
//						unreachableNodesFromNode.get(pickup.number).add(pickup2.number);
//						continue;
					}
//					time+=pickup2.weight *inputdata.timeTonService+inputdata.getTime(pickup2, delivery2);
//					if(time>delivery2.lateTimeWindow) {
//						System.out.println("node "+pickup.number+" and "+pickup2.number);
//						unreachableNodesFromNode.get(pickup.number).add(pickup2.number);
//						continue;
//					}
					
					
					time = delivery2.lateTimeWindow-(pickup.weight*inputdata.timeTonService)-
							inputdata.getTime(pickup, delivery2); // Checking if going from pickup 1 to delivery 2 violates TW in delivery 2
					unreachableDelNodesFromNode.get(pickup.number)[delivery2.number] = time;
					if(time<pickup.lateTimeWindow) {
						//System.out.println("node "+pickup.number+" and "+delivery2.number);
						//System.out.println(time+" < "+pickup.lateTimeWindow);
//						unreachableNodesFromNode.get(pickup.number).add(pickup2.number);
//						continue;
					}
//					time = delivery.earlyTimeWindow+pickup.weight *inputdata.timeTonService+
//							inputdata.getTime(delivery, pickup2);
//					if(time>pickup2.lateTimeWindow) {
//						System.out.println("node "+delivery.number+" and "+pickup2.number);
//						unreachableNodesFromNode.get(delivery.number).add(pickup2.number);
//						continue;
//					}
//					time+=pickup2.weight *inputdata.timeTonService+inputdata.getTime(pickup2, delivery2);
//					if(time>nodes.get(pickup2.number+1).lateTimeWindow) {
//						System.out.println("node "+delivery.number+" and "+pickup2.number);
//						unreachableNodesFromNode.get(delivery.number).add(pickup2.number);
//						continue;
//					}		
//							
				}
			}
			
		}
		
		
		
	}
	public void unreachableDeliveryPairs() {
		unreachableDelPairs = new Hashtable<String, Float>();
		
		for(Node node : nodes) {
		for(Node delivery : deliveryNodes) {
			for(Node delivery2 : deliveryNodes) {
				if(delivery!=delivery2 && delivery!=node && delivery2!=node && delivery.number <delivery2.number ) {
					
					
//					System.out.println("checking for node "+pickup.number+" and "+pickup2.number);
//					Node delivery2 = nodes.get(pickup2.number+1);
					float time = delivery2.lateTimeWindow-(delivery2.weight*inputdata.timeTonService)-
							inputdata.getTime(delivery, delivery2) - (delivery.weight*inputdata.timeTonService)-
							inputdata.getTime(node, delivery); // If going from any node to delivery 1, and then to delivery 2 violates TW in delivery 2
					float time2 = delivery.lateTimeWindow-(delivery.weight*inputdata.timeTonService)-
							inputdata.getTime(delivery2, delivery) - (delivery2.weight*inputdata.timeTonService)-
							inputdata.getTime(node, delivery2); // If going from any node to delivery 2, and then to delivery 1 violates TW in delivery 1
//					float time2= delivery2.lateTimeWindow-pickup2.weight *inputdata.timeTonService+inputdata.getTime(pickup2, delivery2)-
//							Math.max(time, pickup2.earlyTimeWindow);
					String temp = "";
					if(node.number<10) {
						temp+="0"+node.number;
					}
					else {
						temp+=node.number;
					}
					if(delivery.number<10) {
						temp+="0"+delivery.number;
					}
					else {
						temp+=delivery.number;
					}
					if(delivery2.number<10) {
						temp+="0"+delivery2.number;
					}
					else {
						temp+=delivery2.number;
					}
					unreachableDelPairs.put(temp, Math.max(time, time2));
					if(node.lateTimeWindow>Math.max(time, time2)) {
						//System.out.println("at least one ");
						//System.out.println(temp+" "+Math.max(time, time2)+ " "+node.lateTimeWindow );
						//System.out.println(delivery2.lateTimeWindow+" - "+(delivery2.weight*inputdata.timeTonService)+ " - "+
						//	inputdata.getTime(delivery, delivery2)+" - "+(delivery.weight*inputdata.timeTonService)+" - "+
						//	inputdata.getTime(node, delivery));
						//System.out.println(delivery.lateTimeWindow+" - "+(delivery.weight*inputdata.timeTonService)+ " - "+
						//		inputdata.getTime(delivery2, delivery)+" - "+(delivery2.weight*inputdata.timeTonService)+" - "+
						//		inputdata.getTime(node, delivery2));
					}
					
				}
				
			}
			}
		}
		
	}
	

}
