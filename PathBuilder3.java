import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;
import java.util.Vector;

public class PathBuilder3 {
	public Vector<Node> nodes;
	public Vector<Node> pickupNodes;
	public Vector<Node> deliveryNodes;
	public Vector<Node> depot;
	public InstanceData inputdata;
	public PrintWriter pw;
	private double zeroTol = 0.001;
	private int numberOfDominatedLabels;
	public Preprocessing preprocess;
	
	
	public PathBuilder3(Vector<Node> pickupNodes, Vector<Node> deliveryNodes, Vector<Node> nodes, Vector<Node> depot, InstanceData inputdata, PrintWriter pw) {
		this.pickupNodes = pickupNodes;
		this.nodes = nodes;
		this.deliveryNodes = deliveryNodes;
		this.depot = depot;
		this.inputdata = inputdata;
		this.pw = pw;
		numberOfDominatedLabels = 0;
		
		preprocess = new Preprocessing(pickupNodes, deliveryNodes, nodes, depot, inputdata);
		preprocess.unreachableNodeCombination();
		preprocess.unreachableDeliveryNode();
		preprocess.unreachableDeliveryPairs();
	}
	
	
	
	
	public Label LabelExtension(Node node, Label L) {
		
		// Cannot return to start depot
		if(node.number == 0){
			return null;
		}
		
		// Cannot leave end depot
		if (L.node.number == 1){
			return null;
		}
		
		// Computing total daily driving time
		float dailyDrivingTime = L.dailyDrivingTime + inputdata.getTime(L.node, node);
		float startTimeDailyRest = L.startTimeDailyRest;
		int numberDailyRests = L.numberDailyRests;
		float consecutiveDrivingTime = L.consecutiveDrivingTime + inputdata.getTime(L.node, node);
		int totalDistance = L.totalDistance + inputdata.getDistance(L.node, node);
		float startTimeIntermediateBreak = L.startTimeIntermediateBreak;
		float workingTime = L.consecutiveWorkingTime + inputdata.getTime(L.node, node) + L.node.weight*inputdata.timeTonService;
		
	
		// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
		float arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService, node.earlyTimeWindow); 	
		
		//4,5 hour driving rule
		if (consecutiveDrivingTime > 4.5) { // || workingTime > 6
			return null;
		}
		
		
		// If the restrictions on daily driving time (9 hours) or the limit of 24 hours without a daily rest are not met, do not extend the label
		if(dailyDrivingTime > 9 || arrivalTime >13 + 24*(numberDailyRests-1) ) {  //arrivalTime - 11 - startTimeDailyRest > 24
			return null;
		}
		
		if(numberDailyRests == 1 && arrivalTime > 13) {
			return null;
		}
		
	//	if(arrivalTime - startTimeDailyRest > 24) {
	//		return null;
	//	}	
		
		// If the time is greater than the late time window of a node, return null
		if(arrivalTime> node.lateTimeWindow){
			return null;
		}
	
		// Run preprocessing on the nodes in the open nodes set
		for(int i : L.openNodes) {
			if(arrivalTime-zeroTol > preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]) {
//				System.out.println(arrivalTime +"less than unreach: "+preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]);
//				System.out.println(node.number+" "+(i+1));
//				System.exit(0);
				return null;
			}
		}
		
		
		
		if(node.type == "Depot") {
			// Cannot arrive at end depot without delivering every pickup that is picked up
			if(!L.openNodes.isEmpty()){
				return null;	
			}
			Label L2 = new Label();
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = workingTime;
	
			L2.unreachablePickupNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			L2.openNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Calculating profit in the depot node
			L2.profit = L.profit 
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
			return L2;
		}
		
	
		
		
		else if(node.type == "PickupNode"){
			// Returns null if the node is unreachable 
			if(L.unreachablePickupNodes.contains(node.number)) {
				return null;
			}
			
			Label L2 = new Label();
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.time =arrivalTime;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = workingTime;
			
			// Adding the weight corresponding to a pickup node if the pickup node is visited and there is sufficient weight capacity on the vehicle 
			if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
				L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
			}
			else{
				return null;
			}
			
			// // Adding the volume corresponding to a pickup node if the pickup node is visited and there is sufficient weight capacity on the vehicle 
			if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
				L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
			}
			else{
				return null;
			}
			
			L2.unreachablePickupNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's unreachable nodes to this label's unreachable nodes
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			L2.openNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Adding the node to the set of unreachable nodes and open nodes
			L2.unreachablePickupNodes.add(node.number); 
			L2.openNodes.add(node.number);

			// Running preprocessing on the label and checking whether the node in unreachable due to time windows
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			
			// Calculating the profit (revenue - costs) when a pickup node is visited 
			L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, nodes)))
							- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
							- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
							- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
							- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			return L2;
		}
	
		
		
		else if(node.type == "DeliveryNode") {
			// Cannot visit a delivery node whose pickup node has not been visited 
			if (!L.openNodes.contains((node.number-1))){	
				return null;
			}
			
			Label L2 = new Label();
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = workingTime;
			
			
			L2.unreachablePickupNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			L2.openNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Remove the node's corresponding pickup node from the open nodes list when the delivery node i visited
			if (L.openNodes.contains(node.getCorrespondingNode(node, nodes).number)){
				L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, nodes).number));
			}
			
			// Removing the weight corresponding to a delivery node when the delivery node is visited
			L2.weightCapacityUsed = L.weightCapacityUsed - node.weight;
			
			// Removing the volume corresponding to a delivery node when the delivery node is visited
			L2.volumeCapacityUsed = L.volumeCapacityUsed - node.volume;
			
			
			// Calculating the profit when a pickup node is visited (visiting a delivery node only creates costs)
			L2.profit = L.profit 
					- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
					- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
					- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
					- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			
			// Running preprocessing on the label and checking whether the node in unreachable due to time windows
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			return L2;
		}
				
	
		
//		L2.path.add(node.number);
		return null;
	}
	
	
	
public Label LabelExtensionWithDailyRest(Node node, Label L) {
		
		// Returns null if the node is already visited
//		if(L.path.contains(node.number)) {
//			return null;
//		}		
		
		
		
		// Cannot return to start depot
		if(node.number == 0){
			return null;
		}
		
		// Cannot leave end depot
		if (L.node.number == 1){
			return null;
		}
		
	
		//float dailyDrivingTime = L.drivingTime + inputdata.getTime(L.node, node);
		int dailyRestTime = 11; 
		float startTimeDailyRest = L.startTimeDailyRest;
		float startTimeDailyRestDay = 0;
		float dailyDrivingTimeDay = 0;
		//float timeLeftDailyRest = 0;
		float consecutiveDrivingTime = L.consecutiveDrivingTime;
		float startTimeIntermediateBreak = L.startTimeIntermediateBreak;
		float workingTime = L.consecutiveWorkingTime + inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService;
		
		int totalDistance = L.totalDistance + inputdata.getDistance(L.node, node);
		

		// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
		//float arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService, node.earlyTimeWindow); 	
		
	
		float arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime, node.earlyTimeWindow); 
		float timeLeft = 9 - L.dailyDrivingTime;
		float startTimeDailyRestDriving = Math.min(arrivalTime - dailyRestTime, arrivalTime - dailyRestTime - (inputdata.getTime(L.node, node) - timeLeft));
		float dailyDrivingTimeDriving = arrivalTime - dailyRestTime - startTimeDailyRestDriving; //how long driven since last break
		float loadingTimeLeft = 0;
		
		
		int numberDailyRests = L.numberDailyRests;
		//startTimeDailyRest = arrivalTime - dailyRestTime; // - inputdata.getTime(L.node, node); //- L.node.weight*inputdata.timeTonService;
		
		if(numberDailyRests == 1 && arrivalTime - dailyRestTime <= 13) {
			startTimeDailyRestDay = arrivalTime - dailyRestTime;
			dailyDrivingTimeDay = 0;
			consecutiveDrivingTime = 0;
			workingTime = 0;
		} 
		
		else if(numberDailyRests == 1 && arrivalTime - dailyRestTime - startTimeDailyRest > 13 && arrivalTime - dailyRestTime - startTimeDailyRest < 24) {
			startTimeDailyRestDay = 13;
			//loadingTimeLeft =  L.node.weight*inputdata.timeTonService - startTimeDailyRestDay + L.time;
			//if(loadingTimeLeft > 0) {
				dailyDrivingTimeDay = 0; ///L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - dailyRestTime - startTimeDailyRestDay - loadingTimeLeft;//Math.max(0, L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - dailyRestTime - startTimeDailyRestDay);
				consecutiveDrivingTime = 0;
				workingTime = 0;
				//	}
			//	else {
				//	dailyDrivingTimeDay = L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - dailyRestTime - startTimeDailyRestDay;
			//	}
			//dailyDrivingTimeDay = L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - dailyRestTime - startTimeDailyRestDay;// - loadingTimeLeft;
		}
		
		
		if(numberDailyRests > 1 && arrivalTime - dailyRestTime - startTimeDailyRest <= 24) {
			startTimeDailyRestDay = arrivalTime - dailyRestTime;
			dailyDrivingTimeDay = 0;
			consecutiveDrivingTime = 0;
			workingTime = 0;
		}
		
		//if(arrivalTime <= 24 * numberDailyRests) {//(arrivalTime - dailyRestTime - startTimeDailyRest < 24) {
			//startTimeDailyRestDay = arrivalTime - dailyRestTime; // - inputdata.getTime(L.node, node); //- L.node.weight*inputdata.timeTonService;
			//dailyDrivingTimeDay = 0;
			//System.out.println(numberDailyRests);
			//System.out.println("1");
		//}
		
		else if(numberDailyRests > 1 && arrivalTime - dailyRestTime - startTimeDailyRest > 24) {
			startTimeDailyRestDay = L.startTimeDailyRest + 24;
		//	loadingTimeLeft = L.node.weight*inputdata.timeTonService - startTimeDailyRestDay + L.time;
		//	if(loadingTimeLeft > 0) {
			dailyDrivingTimeDay = 0;//L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - dailyRestTime - startTimeDailyRestDay - loadingTimeLeft;//Math.max(0, L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - dailyRestTime - startTimeDailyRestDay);
			consecutiveDrivingTime = 0;
			workingTime = 0;
			//	}
		//	else {
			//	dailyDrivingTimeDay = L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - dailyRestTime - startTimeDailyRestDay;
		//	}
		}
		
		//else if (arrivalTime  > 24*numberDailyRests) {//(arrivalTime - dailyRestTime - startTimeDailyRest >= 24) {
		//	startTimeDailyRestDay =  24*numberDailyRests;
		//	dailyDrivingTimeDay = arrivalTime - dailyRestTime - startTimeDailyRestDriving; //L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - startTimeDailyRest - dailyRestTime; //L.time + inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime  - dailyRestTime - startTimeDailyRestDay;
		//	System.out.println(numberDailyRests);
		//	if (startTimeDailyRestDay > arrivalTime - dailyRestTime && startTimeDailyRest < arrivalTime - dailyRestTime + node.weight*inputdata.timeTonService) {
			//	return null;
			
			
			//else{startTimeDailyRestDay = arrivalTime - dailyRestTime - (arrivalTime - L.startTimeDailyRest - dailyRestTime - 24 ) ;
			//dailyDrivingTimeDay = arrivalTime - dailyRestTime - startTimeDailyRestDay;}
	//	}

		
		//startTimeDailyRest = startTimeDailyRestDriving;
		startTimeDailyRest = Math.min(startTimeDailyRestDriving, startTimeDailyRestDay);
		float dailyDrivingTime = Math.max(dailyDrivingTimeDriving, dailyDrivingTimeDay);
		//float dailyDrivingTime =dailyDrivingTimeDriving;
		consecutiveDrivingTime = dailyDrivingTime; 
		
		
		
		
		
		

		
		
		// If the time is greater than the late time window of a node, return null
		if(arrivalTime> node.lateTimeWindow){
			return null;
		}
	
		
		for(int i : L.openNodes) {
			if(arrivalTime-zeroTol > preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]) {
//				System.out.println(arrivalTime +"less than unreach: "+preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]);
//				System.out.println(node.number+" "+(i+1));
//				System.exit(0);
				return null;
			}
		}
		// Cannot arrive at end depot without delivering every pickup
		
		
			// Removing the pickup node from the open nodes list if its corresponding delivery node is visited
		
	
		if(node.type == "Depot") {
			if(!L.openNodes.isEmpty()){
				return null;	
			}
			Label L2 = new Label();
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests + 1;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = workingTime;
	//	
			
			L2.unreachablePickupNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			L2.openNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			
			
			L2.profit = L.profit 
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
			return L2;
		}
		
	
		// Deciding whether a pickup node is unreachable: if it is visited or if it is unreachable due to its time windows 	
		 if(node.type == "PickupNode"){
			// Returns null if the node is unreachable 
			if(L.unreachablePickupNodes.contains(node.number)) {
				return null;
			}
			
			Label L2 = new Label();
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests + 1;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = workingTime;
			
	
			
			L2.unreachablePickupNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			
			
			// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
//			L2.time = Math.max(L.time+InstanceData.getTime(L.node, node, inputdata)+L.node.weight*inputdata.timeTonService, node.earlyTimeWindow); 	
			L2.time =arrivalTime;
			// If the time is greater than the late time window of a node, return null
			
			// Adding the weight corresponding to a pickup node if the pickup node is visited and there is sufficient weight capacity on the vehicle 
			if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
				L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
			}
			else{
				return null;
			}
			if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
				L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
			}
			else{
				return null;
			}
			
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			L2.openNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			L2.unreachablePickupNodes.add(node.number); 
			L2.openNodes.add(node.number);
//			for (Node pickupNode: pickupNodes){
//				if ( !L2.unreachablePickupNodes.contains(pickupNode.number)){
//					if (L2.time + inputdata.getTime(node, pickupNode) + node.weight *inputdata.timeTonService > pickupNode.lateTimeWindow){
//						L2.unreachablePickupNodes.add(pickupNode.number);
//					}
//					else if (L2.time + inputdata.getTime(node, pickupNode) + node.weight *inputdata.timeTonService +
//							inputdata.getTime(pickupNode, nodes.get(pickupNode.number+1)) + pickupNode.weight *inputdata.timeTonService> nodes.get(pickupNode.number+1).lateTimeWindow){
//						L2.unreachablePickupNodes.add(pickupNode.number);
//					}
//					
//				}
//			}
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
//					System.out.println("addind unreach node");
				}
			}
			// Calculating the profit (revenue - costs) when a pickup node is visited 
			
				L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, nodes)))
							- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
							- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
							- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
							- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
				//System.out.println(L2.profit);
				//System.out.println(L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, nodes), inputdata)));
				//System.out.println(inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node,inputdata));
				//System.out.println(inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node,inputdata));
				//System.out.println(inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node, inputdata));
				//System.out.println((inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time));
				return L2;
		}
	
		
		else if(node.type == "DeliveryNode") {
			// Cannot visit a delivery node whose pickup node has not been visited 
			if (!L.openNodes.contains((node.number-1))){	
				return null;
			}
			Label L2 = new Label();
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests + 1;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = workingTime;
			
	//	
			
			L2.unreachablePickupNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			L2.openNodes = new Vector<Integer>();
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
			L2.time = arrivalTime; 	
			
			// If the time is greater than the late time window of a node, return null
			
			if (L.openNodes.contains(node.getCorrespondingNode(node, nodes).number)){
				L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, nodes).number));
			}
			// Removing the weight corresponding to a delivery node when the delivery node is visited
			L2.weightCapacityUsed = L.weightCapacityUsed - node.weight;
			// Removing the volume corresponding to a delivery node when the delivery node is visited
			L2.volumeCapacityUsed = L.volumeCapacityUsed - node.volume;
			
			L2.profit = L.profit 
					- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
					- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
					- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
					- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			
//			for (Node pickupNode: pickupNodes){
//				if ( !L2.unreachablePickupNodes.contains(pickupNode.number)){
//					if (L2.time + inputdata.getTime(node, pickupNode) + node.weight *inputdata.timeTonService > pickupNode.lateTimeWindow){
//						L2.unreachablePickupNodes.add(pickupNode.number);
//					}
//					else if (L2.time + inputdata.getTime(node, pickupNode) + node.weight *inputdata.timeTonService +
//							inputdata.getTime(pickupNode, nodes.get(pickupNode.number+1)) + pickupNode.weight *inputdata.timeTonService> nodes.get(pickupNode.number+1).lateTimeWindow){
//						L2.unreachablePickupNodes.add(pickupNode.number);
//					}
//					
//				}
//			}
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
//					System.out.println("addind unreach node");
				}
			}
			return L2;
		}
		
		// Adding the volume corresponding to a pickup node if the pickup node is visited and there is sufficient volume capacity on the vehicle 
//		if(node.type=="PickupNode") {
//			
//		}
		
		
		
		
		
		
		
		// Calculating the profit (only costs) when a delivery node or the depots are visited
		
//		L2.path.add(node.number);
		return null;
	}
	

public Label LabelExtensionWithIntermediateBreak(Node node, Label L) {
	
	
	
	
	// Returns null if the node is already visited
//	if(L.path.contains(node.number)) {
//		return null;
//	}		
	
	
	
	// Cannot return to start depot
	if(node.number == 0){
		return null;
	}
	
	// Cannot leave end depot
	if (L.node.number == 1){
		return null;
	}
	
	float dailyDrivingTime = L.dailyDrivingTime + inputdata.getTime(L.node, node);
	//int dailyRestTime = 11; 
	float startTimeDailyRest = L.startTimeDailyRest;
	//float startTimeDailyRestDay = 0;
	//float dailyDrivingTimeDay = 0;
	float startTimeIntermediateBreak = L.startTimeIntermediateBreak ;
	float consecutiveDrivingTime = L.consecutiveDrivingTime;
	float intermediateBreak = Float.parseFloat("0.75");
	float maxDrivingTime = Float.parseFloat("4.5");
	float workingTime = L.consecutiveWorkingTime +  inputdata.getTime(L.node, node) + L.node.weight*inputdata.timeTonService ;
	
	


	//float timeLeftDailyRest = 0;
	
	int totalDistance = L.totalDistance + inputdata.getDistance(L.node, node);
	

	// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
	//float arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService, node.earlyTimeWindow); 	
	

	float arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + intermediateBreak, node.earlyTimeWindow); 
	float timeLeft = maxDrivingTime - L.consecutiveDrivingTime;
	//float timeLeftDriving
	startTimeIntermediateBreak = Math.min(arrivalTime - intermediateBreak, arrivalTime - intermediateBreak - (inputdata.getTime(L.node, node) - timeLeft));
	consecutiveDrivingTime = arrivalTime - intermediateBreak - startTimeIntermediateBreak; //how long driven since last break
	//workingTime = arrivalTime - intermediateBreak - startTimeIntermediateBreak - L.node.weight*inputdata.timeTonService;
	//float loadingTimeLeft = 0;
	
	int numberDailyRests = L.numberDailyRests;
	
	
	//if(arrivalTime - startTimeDailyRest > 24) {
	//	return null;
	//}	
	
	
	// If the restrictions on daily driving time (9 hours) or the limit of 24 hours without a daily rest are not met, do not extend the label
	if(dailyDrivingTime > 9 || arrivalTime-intermediateBreak >13 + 24*(numberDailyRests-1) ) {  //arrivalTime - 11 - startTimeDailyRest > 24
		return null;
	}
			
	if(numberDailyRests == 1 && arrivalTime-intermediateBreak > 13) {
		return null;
	}
	//startTimeDailyRest = arrivalTime - dailyRestTime; // - inputdata.getTime(L.node, node); //- L.node.weight*inputdata.timeTonService;
	
	/* if(numberDailyRests == 1 && arrivalTime - dailyRestTime <= 13) {
		startTimeDailyRestDay = arrivalTime - dailyRestTime;
		dailyDrivingTimeDay = 0;
	} 
	
	else if(numberDailyRests == 1 && arrivalTime - dailyRestTime - startTimeDailyRest > 13 && arrivalTime - dailyRestTime - startTimeDailyRest < 24) {
		startTimeDailyRestDay = 13;
		//loadingTimeLeft =  L.node.weight*inputdata.timeTonService - startTimeDailyRestDay + L.time;
		//if(loadingTimeLeft > 0) {
			dailyDrivingTimeDay = 0; ///L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - dailyRestTime - startTimeDailyRestDay - loadingTimeLeft;//Math.max(0, L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - dailyRestTime - startTimeDailyRestDay);
		//	}
		//	else {
			//	dailyDrivingTimeDay = L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - dailyRestTime - startTimeDailyRestDay;
		//	}
		//dailyDrivingTimeDay = L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - dailyRestTime - startTimeDailyRestDay;// - loadingTimeLeft;
	}
	
	
	if(numberDailyRests > 1 && arrivalTime - dailyRestTime - startTimeDailyRest <= 24) {
		startTimeDailyRestDay = arrivalTime - dailyRestTime;
		dailyDrivingTimeDay = 0;
	}
	
	//if(arrivalTime <= 24 * numberDailyRests) {//(arrivalTime - dailyRestTime - startTimeDailyRest < 24) {
		//startTimeDailyRestDay = arrivalTime - dailyRestTime; // - inputdata.getTime(L.node, node); //- L.node.weight*inputdata.timeTonService;
		//dailyDrivingTimeDay = 0;
		//System.out.println(numberDailyRests);
		//System.out.println("1");
	//}
	
	else if(numberDailyRests > 1 && arrivalTime - dailyRestTime - startTimeDailyRest > 24) {
		startTimeDailyRestDay = L.startTimeDailyRest + 24;
	//	loadingTimeLeft = L.node.weight*inputdata.timeTonService - startTimeDailyRestDay + L.time;
	//	if(loadingTimeLeft > 0) {
		dailyDrivingTimeDay = 0;//L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - dailyRestTime - startTimeDailyRestDay - loadingTimeLeft;//Math.max(0, L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - dailyRestTime - startTimeDailyRestDay);
	//	}
	//	else {
		//	dailyDrivingTimeDay = L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - dailyRestTime - startTimeDailyRestDay;
	//	}
	}
	
	//else if (arrivalTime  > 24*numberDailyRests) {//(arrivalTime - dailyRestTime - startTimeDailyRest >= 24) {
	//	startTimeDailyRestDay =  24*numberDailyRests;
	//	dailyDrivingTimeDay = arrivalTime - dailyRestTime - startTimeDailyRestDriving; //L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime - startTimeDailyRest - dailyRestTime; //L.time + inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime  - dailyRestTime - startTimeDailyRestDay;
	//	System.out.println(numberDailyRests);
	//	if (startTimeDailyRestDay > arrivalTime - dailyRestTime && startTimeDailyRest < arrivalTime - dailyRestTime + node.weight*inputdata.timeTonService) {
		//	return null;
		
		
		//else{startTimeDailyRestDay = arrivalTime - dailyRestTime - (arrivalTime - L.startTimeDailyRest - dailyRestTime - 24 ) ;
		//dailyDrivingTimeDay = arrivalTime - dailyRestTime - startTimeDailyRestDay;}
//	} 
	
	//startTimeDailyRest = startTimeDailyRestDriving;
	startTimeDailyRest = Math.min(startTimeDailyRestDriving, startTimeDailyRestDay);
	float dailyDrivingTime = Math.max(dailyDrivingTimeDriving, dailyDrivingTimeDay);
	//float dailyDrivingTime =dailyDrivingTimeDriving;
	
*/	
	
	
	
	

	
	
	// If the time is greater than the late time window of a node, return null
	if(arrivalTime> node.lateTimeWindow){
		return null;
	}

	
	for(int i : L.openNodes) {
		if(arrivalTime-zeroTol > preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]) {
//			System.out.println(arrivalTime +"less than unreach: "+preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]);
//			System.out.println(node.number+" "+(i+1));
//			System.exit(0);
			return null;
		}
	}
	// Cannot arrive at end depot without delivering every pickup
	
	
		// Removing the pickup node from the open nodes list if its corresponding delivery node is visited
	

	if(node.type == "Depot") {
		if(!L.openNodes.isEmpty()){
			return null;	
		}
		Label L2 = new Label();
		L2.node = node;
		L2.predesessor = L;
		L2.time = arrivalTime;
		L2.dailyDrivingTime = dailyDrivingTime;
		L2.startTimeDailyRest = startTimeDailyRest;
		L2.numberDailyRests = numberDailyRests;
		L2.totalDistance = totalDistance;
		L2.consecutiveDrivingTime = consecutiveDrivingTime;
		L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
		L2.consecutiveWorkingTime = workingTime;
//	
		
		L2.unreachablePickupNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
		for(int i : L.unreachablePickupNodes) {
			L2.unreachablePickupNodes.add(i);
		}
		
		L2.openNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's openNodes to this label's openNodes
		for(int i : L.openNodes) {
			L2.openNodes.add(i);
		}
		
		
		
		L2.profit = L.profit 
					- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
					- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
					- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
					- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
		return L2;
	}
	

	// Deciding whether a pickup node is unreachable: if it is visited or if it is unreachable due to its time windows 	
	 if(node.type == "PickupNode"){
		// Returns null if the node is unreachable 
		if(L.unreachablePickupNodes.contains(node.number)) {
			return null;
		}
		
		Label L2 = new Label();
		L2.node = node;
		L2.predesessor = L;
		L2.time = arrivalTime;
		L2.dailyDrivingTime = dailyDrivingTime;
		L2.startTimeDailyRest = startTimeDailyRest;
		L2.numberDailyRests = numberDailyRests;
		L2.totalDistance = totalDistance;
		L2.consecutiveDrivingTime = consecutiveDrivingTime;
		L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
		L2.consecutiveWorkingTime = workingTime;
		
		

		
		L2.unreachablePickupNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
		
		
		// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
//		L2.time = Math.max(L.time+InstanceData.getTime(L.node, node, inputdata)+L.node.weight*inputdata.timeTonService, node.earlyTimeWindow); 	
		L2.time =arrivalTime;
		// If the time is greater than the late time window of a node, return null
		
		// Adding the weight corresponding to a pickup node if the pickup node is visited and there is sufficient weight capacity on the vehicle 
		if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
			L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
		}
		else{
			return null;
		}
		if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
			L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
		}
		else{
			return null;
		}
		
		for(int i : L.unreachablePickupNodes) {
			L2.unreachablePickupNodes.add(i);
		}
		
		L2.openNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's openNodes to this label's openNodes
		for(int i : L.openNodes) {
			L2.openNodes.add(i);
		}
		
		L2.unreachablePickupNodes.add(node.number); 
		L2.openNodes.add(node.number);
//		for (Node pickupNode: pickupNodes){
//			if ( !L2.unreachablePickupNodes.contains(pickupNode.number)){
//				if (L2.time + inputdata.getTime(node, pickupNode) + node.weight *inputdata.timeTonService > pickupNode.lateTimeWindow){
//					L2.unreachablePickupNodes.add(pickupNode.number);
//				}
//				else if (L2.time + inputdata.getTime(node, pickupNode) + node.weight *inputdata.timeTonService +
//						inputdata.getTime(pickupNode, nodes.get(pickupNode.number+1)) + pickupNode.weight *inputdata.timeTonService> nodes.get(pickupNode.number+1).lateTimeWindow){
//					L2.unreachablePickupNodes.add(pickupNode.number);
//				}
//				
//			}
//		}
		for(Node pickup: pickupNodes) {
			if(!L2.unreachablePickupNodes.contains(pickup.number)) {
				if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
				L2.unreachablePickupNodes.add(pickup.number);
//				System.out.println("addind unreach node");
			}
		}
		// Calculating the profit (revenue - costs) when a pickup node is visited 
		
			L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, nodes)))
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			//System.out.println(L2.profit);
			//System.out.println(L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, nodes), inputdata)));
			//System.out.println(inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node,inputdata));
			//System.out.println(inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node,inputdata));
			//System.out.println(inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node, inputdata));
			//System.out.println((inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time));
			return L2;
	}

	
	else if(node.type == "DeliveryNode") {
		// Cannot visit a delivery node whose pickup node has not been visited 
		if (!L.openNodes.contains((node.number-1))){	
			return null;
		}
		Label L2 = new Label();
		L2.node = node;
		L2.predesessor = L;
		L2.time = arrivalTime;
		L2.dailyDrivingTime = dailyDrivingTime;
		L2.startTimeDailyRest = startTimeDailyRest;
		L2.numberDailyRests = numberDailyRests;
		L2.totalDistance = totalDistance;
		L2.consecutiveDrivingTime = consecutiveDrivingTime;
		L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
		L2.consecutiveWorkingTime = workingTime;
//	
		
		L2.unreachablePickupNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
		for(int i : L.unreachablePickupNodes) {
			L2.unreachablePickupNodes.add(i);
		}
		
		L2.openNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's openNodes to this label's openNodes
		for(int i : L.openNodes) {
			L2.openNodes.add(i);
		}
		
		// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
		L2.time = arrivalTime; 	
		
		// If the time is greater than the late time window of a node, return null
		
		if (L.openNodes.contains(node.getCorrespondingNode(node, nodes).number)){
			L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, nodes).number));
		}
		// Removing the weight corresponding to a delivery node when the delivery node is visited
		L2.weightCapacityUsed = L.weightCapacityUsed - node.weight;
		// Removing the volume corresponding to a delivery node when the delivery node is visited
		L2.volumeCapacityUsed = L.volumeCapacityUsed - node.volume;
		
		L2.profit = L.profit 
				- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
				- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
				- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
				- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
		
//		for (Node pickupNode: pickupNodes){
//			if ( !L2.unreachablePickupNodes.contains(pickupNode.number)){
//				if (L2.time + inputdata.getTime(node, pickupNode) + node.weight *inputdata.timeTonService > pickupNode.lateTimeWindow){
//					L2.unreachablePickupNodes.add(pickupNode.number);
//				}
//				else if (L2.time + inputdata.getTime(node, pickupNode) + node.weight *inputdata.timeTonService +
//						inputdata.getTime(pickupNode, nodes.get(pickupNode.number+1)) + pickupNode.weight *inputdata.timeTonService> nodes.get(pickupNode.number+1).lateTimeWindow){
//					L2.unreachablePickupNodes.add(pickupNode.number);
//				}
//				
//			}
//		}
		for(Node pickup: pickupNodes) {
			if(!L2.unreachablePickupNodes.contains(pickup.number)) {
				if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
				L2.unreachablePickupNodes.add(pickup.number);
//				System.out.println("addind unreach node");
			}
		}
		return L2;
	}
	
	// Adding the volume corresponding to a pickup node if the pickup node is visited and there is sufficient volume capacity on the vehicle 
//	if(node.type=="PickupNode") {
//		
//	}
	
	
	
	
	
	
	
	// Calculating the profit (only costs) when a delivery node or the depots are visited
	
//	L2.path.add(node.number);
	return null;
}




	
	
	public Vector<Label> BuildPaths() {
		Vector<Label> list = new Vector<Label>();  // List of non-dominated labels 
		Label L = new Label();
		// Initializing label
		L.labelNumber = 0;
//		L.path = new Vector<Integer>();
		L.node = nodes.get(0);
		L.time = Float.parseFloat("0");
		L.profit = 0;
		L.weightCapacityUsed = 0;
		L.volumeCapacityUsed = 0;
		L.numberDailyRests = 1;
		L.predesessor = null;
		L.totalDistance = 0;
		L.startTimeDailyRest = 0;
		L.unreachablePickupNodes = new Vector<Integer>();
		L.openNodes = new Vector<Integer>();		
		L.startTimeIntermediateBreak = 0;
		L.consecutiveDrivingTime = 0;
		L.consecutiveWorkingTime = 0;
//		L.path.add(L.node.number);
		ArrayList<Vector<Label>> unprocessedAtNode = new ArrayList<Vector<Label>>();
		ArrayList<Vector<Label>> processedAtNode = new ArrayList<Vector<Label>>();
		for(int i = 0; i < nodes.size(); i++) {
			Vector<Label> processed = new Vector<Label>();
			processedAtNode.add(i, processed);
			Vector<Label> unprocessed = new Vector<Label>();
			unprocessedAtNode.add(i, unprocessed);
		}
		PriorityQueue<Label> unprocessedQueue = new PriorityQueue<Label>(5, new UnprocessedComparator()); 
		unprocessedQueue.add(L);
		
		int counter =0;
		//Going through all unprocessed labels
		while(!unprocessedQueue.isEmpty()) { 
			Label label = unprocessedQueue.remove();
			counter++;
			if(counter%1000 == 0) {
				//System.out.println(counter+" "+label.toString());
				//System.out.println("number of unprocessed labels: "+unprocessedQueue.size());
			}
//			for(int i = 2; i < nodes.size(); i++) { // Going through all nodes except node 0 and node 1 (the depot nodes)
//				Label newLabel = LabelExtension(nodes.get(i), label);
//				if(newLabel!=null) {
//					if(checkdominance(newLabel, unprocessedQueue, unprocessedAtNode.get(newLabel.node.number), processedAtNode.get(newLabel.node.number))) {
//						unprocessedQueue.add(newLabel); 
//						unprocessedAtNode.get(newLabel.node.number).add(newLabel);
//					}
//				}
//			}
			for(Node pickup :pickupNodes) { // Going through all nodes except node 0 and node 1 (the depot nodes)
				Label newLabel = LabelExtension(pickup, label);
				
				if(newLabel!=null) {
					//System.out.println(newLabel.toString());
					if(checkdominance(newLabel, unprocessedQueue, unprocessedAtNode.get(newLabel.node.number), processedAtNode.get(newLabel.node.number))) {
						unprocessedQueue.add(newLabel); 
						unprocessedAtNode.get(newLabel.node.number).add(newLabel);
					}
				}
				Label newLabel2 = LabelExtensionWithDailyRest(pickup, label);
				
				if(newLabel2!=null) {
					//System.out.println(newLabel2.toString());
					if(checkdominance(newLabel2, unprocessedQueue, unprocessedAtNode.get(newLabel2.node.number), processedAtNode.get(newLabel2.node.number))) {
						unprocessedQueue.add(newLabel2); 
						unprocessedAtNode.get(newLabel2.node.number).add(newLabel2);
					}
				}
				
				Label newLabel3 = LabelExtensionWithIntermediateBreak(pickup, label);
				
				if(newLabel3!=null) {
					//System.out.println(newLabel2.toString());
					if(checkdominance(newLabel3, unprocessedQueue, unprocessedAtNode.get(newLabel3.node.number), processedAtNode.get(newLabel3.node.number))) {
						unprocessedQueue.add(newLabel3); 
						unprocessedAtNode.get(newLabel3.node.number).add(newLabel3);
					}
				}
				
			}
			for(int i : label.openNodes) { // Going through all nodes except node 0 and node 1 (the depot nodes)
				Label newLabel = LabelExtension(nodes.get(i+1), label);
				
				if(newLabel!=null) {
					//System.out.println(newLabel.toString());
					if(checkdominance(newLabel, unprocessedQueue, unprocessedAtNode.get(newLabel.node.number), processedAtNode.get(newLabel.node.number))) {
						unprocessedQueue.add(newLabel); 
						unprocessedAtNode.get(newLabel.node.number).add(newLabel);
					}
				}
				Label newLabel2 = LabelExtensionWithDailyRest(nodes.get(i+1), label);
				
				if(newLabel2!=null) {
					//System.out.println(newLabel2.toString());
					if(checkdominance(newLabel2, unprocessedQueue, unprocessedAtNode.get(newLabel2.node.number), processedAtNode.get(newLabel2.node.number))) {
						unprocessedQueue.add(newLabel2); 
						unprocessedAtNode.get(newLabel2.node.number).add(newLabel2);
					}
				}
				
				Label newLabel3 = LabelExtensionWithIntermediateBreak(nodes.get(i+1), label);
				
				if(newLabel3!=null) {
					//System.out.println(newLabel2.toString());
					if(checkdominance(newLabel3, unprocessedQueue, unprocessedAtNode.get(newLabel3.node.number), processedAtNode.get(newLabel3.node.number))) {
						unprocessedQueue.add(newLabel3); 
						unprocessedAtNode.get(newLabel3.node.number).add(newLabel3);
					}
				}
			}
			Label newLabel = LabelExtension(nodes.get(1), label); // Adding node 1 (the end depot node) to the end of the path 
			
			if(newLabel!=null) {
				//System.out.println(newLabel.toString());
				if(checkdominance(newLabel, unprocessedQueue, unprocessedAtNode.get(newLabel.node.number), processedAtNode.get(newLabel.node.number))) {
					list.add(newLabel);
				}
			}
			Label newLabel2 = LabelExtensionWithDailyRest(nodes.get(1), label);
			
			if(newLabel2!=null) {
			//	System.out.println(newLabel2.toString());
				if(checkdominance(newLabel2, unprocessedQueue, unprocessedAtNode.get(newLabel2.node.number), processedAtNode.get(newLabel2.node.number))) {
					list.add(newLabel2);
				}
			}
			
			Label newLabel3 = LabelExtensionWithIntermediateBreak(nodes.get(1), label);
			
			if(newLabel3!=null) {
				//System.out.println(newLabel2.toString());
				if(checkdominance(newLabel3, unprocessedQueue, unprocessedAtNode.get(newLabel3.node.number), processedAtNode.get(newLabel3.node.number))) {
					unprocessedQueue.add(newLabel3); 
					unprocessedAtNode.get(newLabel3.node.number).add(newLabel3);
				}
			}
			
			processedAtNode.get(label.node.number).add(label); // The label removed from unprocessed is added to processed
		}
		
//		System.out.println("Number of paths:" + processed.size());
		System.out.println("number of non-dominated paths: "+list.size());
		pw.println("number of non-dominated paths: "+list.size());
		System.out.println("number of dominated labels: "+numberOfDominatedLabels);
		pw.println("number of dominated labels: "+numberOfDominatedLabels);
		System.out.println("The best label is:");
		pw.println ("The best label is: ");
		System.out.println(findBestLabel(list).toString());
		//pw.println(findBestLabel(list).toString());
		//for(Label i : list) {
		//System.out.println(i.toString());
		//}
		return list;
	}
	
	
	private boolean dominateLabel(Label L1, Label L2) {
		if(L1.time-zeroTol<=L2.time && L1.profit+zeroTol>=L2.profit && L1.node.number == L2.node.number && L1.startTimeDailyRest >= L2.startTimeDailyRest && L1.dailyDrivingTime <= L2.dailyDrivingTime && L1.startTimeIntermediateBreak >= L2.startTimeIntermediateBreak &&  L1.consecutiveDrivingTime <= L2.consecutiveDrivingTime && L1.consecutiveWorkingTime <= L2.consecutiveWorkingTime) { //
			for (int i : L1.openNodes ){
				if (!L2.openNodes.contains(i)){
					return false;
				}
			}
			for (int i : L1.unreachablePickupNodes ){
				if (!L2.unreachablePickupNodes.contains(i)){
					return false;
				}
			}
			//System.out.println("Label: ");
			//System.out.println(L1.toString());
			//System.out.println("dominates label: ");
			//System.out.println(L2.toString());
			return true;
		}
		else return false; 
	}
	
	
	
	//Updates the processed and unprocessed lists according to the dominated labels.
	private boolean checkdominance(Label newLabel, PriorityQueue<Label> unprocessedQueue, Vector<Label> unprocessed, Vector<Label> processed) {
		Vector<Label> remove = new Vector<Label>();
		
		for(Label oldLabel : unprocessed) {
			if(dominateLabel(oldLabel, newLabel)) {
				numberOfDominatedLabels++;
				unprocessedQueue.removeAll(remove);
				unprocessed.removeAll(remove);
				return false;
			}
			else if(dominateLabel(newLabel,oldLabel)) {
				remove.add(oldLabel);
				numberOfDominatedLabels++;
			}
		}
		unprocessedQueue.removeAll(remove);
		unprocessed.removeAll(remove);
		
		remove = new Vector<Label>();
		for(Label oldLabel : processed) {
			if(dominateLabel(oldLabel, newLabel)) {
				processed.removeAll(remove);
				numberOfDominatedLabels++;
				return false;
			}
			else if(dominateLabel(newLabel,oldLabel)) {
				numberOfDominatedLabels++;
				remove.add(oldLabel);
			}
		}
		processed.removeAll(remove);
		
		return true;
	}
	
	public Label findBestLabel(Vector<Label> list) {
		float currentBestProfit = 0;
		Label bestLabel = null;
		for(Label i : list) {
			if(i.profit > currentBestProfit) {
				currentBestProfit = i.profit;
				bestLabel = i;
			}
		}
		
		Label temp = bestLabel.predesessor;
		while(temp!=null) {
			System.out.println(temp.toString());
			pw.println(temp.toString());
		temp=temp.predesessor;
		} 
		pw.println(bestLabel.toString());
		return bestLabel;
		
	}
	


}