import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;
import java.util.PriorityQueue;
import java.util.Vector;

import sun.security.tools.PathList;


public class PathBuilder {
	//public Vector<Node> nodes;
	public Vector<Node> pickupNodes;
	public Vector<Node> deliveryNodes;
	public Vector<Node> nodesWithoutDepot;
	//public Vector<Node> depot;
	public Vector<Node> startDepots;
	public Vector <Vehicle> vehicles;
	//public Vector<Route> routes;
	public InstanceData inputdata;
	public PrintWriter pw;
	public Preprocessing preprocess;
	//public Vehicle vehicle;
	private double zeroTol = 0.001;
	public int numberOfDominatedLabels;
	public int numberOfPaths; 
	int routeNumber = 1;
//	public double[] dualVisitedPickupsCon;
//	public Vector<double> dualOneVisitCon;
	//int bestLabelNumber = 0;


	public PathBuilder(Vector<Node> pickupNodes, Vector<Node> deliveryNodes, InstanceData inputdata, PrintWriter pw, Vector<Vehicle> vehicles) {
	//	this.nodes = nodes;
		this.pickupNodes = pickupNodes;
		this.deliveryNodes = deliveryNodes;
	//	this.depot = depot;
		// this.routes = routes;
		this.inputdata = inputdata;
		this.pw = pw;
		this.vehicles = vehicles;
		numberOfDominatedLabels = 0;
		
	//	preprocess = new Preprocessing(pickupNodes, deliveryNodes, vehicles, inputdata, nodesWithoutDepot);
	//	preprocess.unreachableNodeCombination();
	//	preprocess.unreachableDeliveryNode();
	//	preprocess.unreachableDeliveryPairs();
	}
	
	
	
	// Label extension without any intermediate breaks or daily rests 
	public Label LabelExtension(Node node, Label L, double[] dualVisitedPickupsCon) {
		

		
		// Cannot return to start depot
		if(node.number == 0){
			return null;
		}
		
		// Cannot leave end depot
		if (L.node.number == 1){
			return null;
		}
		
		// Defining rule related values
		int maxDailyDrivingTime = 9;
		int maxConsecutiveWorkingTime = 6;
		double maxConsecutiveDrivingTime = 4.5;
		
		// Computing total daily driving time, consecutive driving time, consecutive working time and total distance
		double dailyDrivingTime = L.dailyDrivingTime + inputdata.getTime(L.node, node);
		double consecutiveDrivingTime = L.consecutiveDrivingTime + inputdata.getTime(L.node, node);
		double consecutiveWorkingTime = L.consecutiveWorkingTime + inputdata.getTime(L.node, node) + L.node.weight*inputdata.timeTonService;
		double totalDistance = L.totalDistance + inputdata.getDistance(L.node, node);
		
		// Setting break and rest related parameters to that equal in the previous node
		double startTimeDailyRest = L.startTimeDailyRest;
		int numberDailyRests = L.numberDailyRests;
		double startTimeIntermediateBreak = L.startTimeIntermediateBreak;
		
	//	System.out.println("TYPE HER" +L.vehicle.nodes.get(node.number).type);
	//	System.out.println("TYPE WEIGHT" +L.vehicle.nodes.get(node.number).weight);
	//	System.out.println("TYPE HER" +L.node.weight);
		// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
		double arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService, node.earlyTimeWindow); 	
		
		// The rule of maximum 4.5 hours consecutive driving and 6 hours consecutive working cannot be broken
		if (consecutiveDrivingTime > maxConsecutiveDrivingTime || consecutiveWorkingTime > maxConsecutiveWorkingTime) {  
			return null;
		}
		
		
		// The rule of maximum 9 hours daily driving and the limit of 24 hours without a daily rest are not met, do not extend the label
		if(dailyDrivingTime > maxDailyDrivingTime || arrivalTime >13 + 24*(numberDailyRests-1) ) {  
			return null;
		}
		
		// If no daily rests are taken (numberDailyRests = 1), then the maximal time before the first daily rest is 13
		if(numberDailyRests == 1 && arrivalTime > 13) {
			return null;
		}
		
		// If the arrival time is greater than the late time window of a node, do not extend the label
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
			
			if(L.node.number == 0) {
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.totalPickupDual = L.totalPickupDual;
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Calculating profit in the depot node
			L2.profit = L.profit 
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;

			return L2;
		}
		
	
		else if(node.type == "PickupNode"){
			
			// Returns null if the node is unreachable 
			if(L.unreachablePickupNodes.contains(node.number)) {
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.node = node;
			L2.pickupDual = dualVisitedPickupsCon[node.number/2 - 1];
			L2.totalPickupDual = L.totalPickupDual + L2.pickupDual;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.time =arrivalTime;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			L2.pickupNodesVisited.add(node.number);
			
			// Adding the weight corresponding to a pickup node if there is sufficient weight capacity on the vehicle 
			if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
				L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
			}
			else{
				return null;
			}
			
			// Adding the volume corresponding to a pickup node if there is sufficient volume capacity on the vehicle
			if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
				L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
			}
			else{
				return null;
			}
			
			// Adding all elements from the predecessor's unreachable nodes to this label's unreachable nodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Adding the node to the sets of unreachable nodes and open nodes
			L2.unreachablePickupNodes.add(node.number); 
			L2.openNodes.add(node.number);

			// Running preprocessing on the label and checking whether the node in unreachable
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			
			// Calculating the profit (revenue - costs) when a pickup node is visited 
			L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, L2.vehicle.nodes)))
							- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
							- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
							- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
							- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
			
			return L2;
		}
	
		
		
		else if(node.type == "DeliveryNode") {
			
			
			// Cannot visit a delivery node whose pickup node has not been visited 
			if (!L.openNodes.contains((node.number-1))){	
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.node = node;
			L2.totalPickupDual = L.totalPickupDual;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			
			
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Remove the node's corresponding pickup node from the open nodes list when the delivery node i visited
			if (L.openNodes.contains(node.getCorrespondingNode(node, L2.vehicle.nodes).number)){
				L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, L2.vehicle.nodes).number));
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
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
			
			// Running preprocessing and checking whether the node in unreachable 
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			return L2;
		}
		return null;
	}
	
	
	// Label extension with one daily rest
	public Label LabelExtensionWithDailyRest(Node node, Label L, double[] dualVisitedPickupsCon) {
			
		// Cannot return to start depot
		if(node.number == 0){
			return null;
		}
		
		// Cannot leave end depot
		if (L.node.number == 1){
			return null;
		}
		
		// Defining rule related values
		int dailyRestTime = 11; 
		int maxDailyDrivingTime = 9;
		int maxConsecutiveWorkingTime = 6;
		double maxConsecutiveDrivingTime = 4.5;
		
		// Setting intermediate break and daily rest related values to those of the previous label
		double startTimeDailyRest = L.startTimeDailyRest;
		double consecutiveDrivingTime = L.consecutiveDrivingTime;
		double startTimeIntermediateBreak = L.startTimeIntermediateBreak;
		double consecutiveWorkingTime = L.consecutiveWorkingTime;
		double dailyDrivingTime = L.dailyDrivingTime ;
		double totalDistance = L.totalDistance + inputdata.getDistance(L.node, node);
		
		// Time in the label equals max of: 1) the predecessor's time plus travel-, service- and daily rest time to this node, 2) early time window in this node
		double arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime, node.earlyTimeWindow); 
		

		
		
		// The driving time required on the arc
		double arcDrivingTime = inputdata.getTime(L.node, node);
		
		// The arrival time without considering the early time window in the next node 
		double arrivalTimeNoWait = L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + dailyRestTime; 
		
		// Computing the time left before reaching the max values of daily driving, consecutive driving, and consecutive working 
		double timeLeftDailyDriving = maxDailyDrivingTime - L.dailyDrivingTime;
		double timeLeftDriving = maxConsecutiveDrivingTime - L.consecutiveDrivingTime;
		double timeLeftWorking = maxConsecutiveWorkingTime - L.consecutiveWorkingTime - L.node.weight*inputdata.timeTonService;
		
		// Time to break is the smallest of the times computed above 
		double timeToBreak = Math.min(timeLeftDailyDriving, timeLeftDriving);
		timeToBreak = Math.min(timeToBreak, timeLeftWorking);
		
		// The start time of the daily rest is the arrival time at the previous node plus the service time at that node and the time to break 
		startTimeDailyRest = L.time + L.node.weight*inputdata.timeTonService + timeToBreak;
		
		// Consecutive driving and working time, and daily driving time are equal to what is left driving on the arc after the daily rest is taken
		consecutiveDrivingTime = arcDrivingTime - timeToBreak;
		consecutiveWorkingTime = arcDrivingTime - timeToBreak ;
		dailyDrivingTime = arcDrivingTime - timeToBreak;   
		
		// If the daily rest must be taken within the service time on the node in order to stay feasible, do not extend the label
		if (timeLeftWorking <= 0) {  
			return null;
		}
		
		// If no daily rest is necessary to stay feasible, place the daily rest at the end of the arc
		if (startTimeDailyRest > L.time + (L.node.weight*inputdata.timeTonService) + arcDrivingTime) { 
			startTimeDailyRest = arrivalTime - dailyRestTime;
			consecutiveWorkingTime = 0;
			consecutiveDrivingTime = 0;
			dailyDrivingTime = 0;
		}
		
		
		// The following code computes the start time of the daily rest in case the 24-hour rule is the one that is most restrictive 
		// The 24-hour rule says that if the time is more than max = 13 + 24 * number of daily rests, then the start time of the daily rest is equal to the max number
		int numberDailyRests = L.numberDailyRests;
		
		// If the number of daily rests is equal to 1, there must be maximal (24-11=) 13 hours to the start of the first daily rest 
		if(numberDailyRests == 1 && startTimeDailyRest > 13 ) { 
			startTimeDailyRest = 13; 
			// Calculating the time driven before the daily rest. Daily driving time, and consecutive driving and working time are set to the arc driving time minus the time driven
			double timeDriven = startTimeDailyRest - L.time - L.node.weight*inputdata.timeTonService;
			dailyDrivingTime = arcDrivingTime - timeDriven; 
			consecutiveDrivingTime = arcDrivingTime - timeDriven; 
			consecutiveWorkingTime = arcDrivingTime - timeDriven;
			// If the daily rest must be taken inside the service time, do not extend the label
			if(timeDriven < 0) {  
				return null;
			}
			// Making sure that the daily rest is always taken on the current arc. If it is not necessary with a daily rest, place it at the end of the arc
			if (arrivalTimeNoWait < node.earlyTimeWindow && timeDriven > arcDrivingTime) {
				dailyDrivingTime = 0;
				consecutiveWorkingTime = 0;
				consecutiveDrivingTime = 0;	
				startTimeDailyRest = arrivalTime - dailyRestTime;
			} 
		}
		
		// If the start time of the daily rest is higher than max = 13 + numberOfDailyRests-1*24
		else if(numberDailyRests > 1 && startTimeDailyRest  > 24 * (numberDailyRests -1) + 13) { 
			startTimeDailyRest = 13 + 24*(numberDailyRests-1);
			// Calculating the time driven before the daily rest. Daily driving time, and consecutive driving and working time are set to the arc driving time minus the time driven
			double timeDriven = startTimeDailyRest - L.time - L.node.weight*inputdata.timeTonService;
			dailyDrivingTime = arcDrivingTime - timeDriven; 
			consecutiveDrivingTime = arcDrivingTime - timeDriven; 
			consecutiveWorkingTime =  arcDrivingTime - timeDriven;
			// If the daily rest must be taken inside the service time, do not extend the label 
			if(timeDriven < 0) { 
				return null;
			}
			// Making sure that the daily rest is always taken on the current arc. If it is not necessary with a daily rest, place it at the end of the arc
			if (arrivalTimeNoWait < node.earlyTimeWindow && timeDriven > arcDrivingTime) { 
				dailyDrivingTime = 0;
				consecutiveWorkingTime = 0;
				consecutiveDrivingTime = 0;	
				startTimeDailyRest = arrivalTime - dailyRestTime;
			} 	
		}
		
		// If the time is greater than the late time window of a node, do not extend the label
		if(arrivalTime> node.lateTimeWindow){
			return null;
		}
		
		// If the consecutive driving time is larger than 4.5 hours or the consecutive working time is larger than 6 hours, do not extend the label
		if (consecutiveDrivingTime > maxConsecutiveDrivingTime || consecutiveWorkingTime > maxConsecutiveWorkingTime) {
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
			
			if(L.node.number == 0) {
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.totalPickupDual = L.totalPickupDual; 
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests + 1;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;	
		
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			
			
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
		
			
			// Calculating profit in the depot node
			L2.profit = L.profit 
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
	
			return L2;
		}
		
		
		if(node.type == "PickupNode"){
			// Returns null if the node is unreachable 
			if(L.unreachablePickupNodes.contains(node.number)) {
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.node = node;
			L2.pickupDual = dualVisitedPickupsCon[node.number/2 - 1];
			L2.totalPickupDual = L.totalPickupDual + L2.pickupDual;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests + 1;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			L2.pickupNodesVisited.add(node.number);
			
			// Adding the weight corresponding to a pickup node if there is sufficient weight capacity on the vehicle 
			if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
				L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
			}
			else{
				return null;
			}
			
			// Adding the volume corresponding to a pickup node if there is sufficient volume capacity on the vehicle 
			if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
				L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
			}
			else{
				return null;
			}
			
			// Adding all elements from the predecessor's unreachable nodes to this label's unreachable nodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Adding the node to the sets of unreachable nodes and open nodes
			L2.unreachablePickupNodes.add(node.number); 
			L2.openNodes.add(node.number);

			// Running preprocessing on the label and checking whether the node in unreachable
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);			
				}
			}
			
			// Calculating the profit (revenue - costs) when a pickup node is visited 
			L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, L2.vehicle.nodes)))
							- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
							- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
							- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
							- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
			
		
			
			
			return L2;
		}
	
		
		else if(node.type == "DeliveryNode") {
			
			// Cannot visit a delivery node whose pickup node has not been visited 
			if (!L.openNodes.contains((node.number-1))){	
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.node = node;
			L2.totalPickupDual = L.totalPickupDual;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests + 1;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			
			
			
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Remove the node's corresponding pickup node from the open nodes list when the delivery node i visited
			if (L.openNodes.contains(node.getCorrespondingNode(node, L2.vehicle.nodes).number)){
				L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, L2.vehicle.nodes).number));
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
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
			
			// Running preprocessing and checking whether the node in unreachable 
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			return L2;
		}
	
		return null;
	}
	
	
	// Label extension with one or two intermediate breaks
	public Label LabelExtensionWithIntermediateBreak(Node node, Label L, double[] dualVisitedPickupsCon) {
			
		// Cannot return to start depot
		if(node.number == 0){
			return null;
		}

		
		// Cannot leave end depot
		if (L.node.number == 1){
			return null;
		}
		
		// Defining rule related values
		double intermediateBreakTime = 0.75;
		double maxConsecutiveDrivingTime = 4.5;
		int maxWorkingTime = 6;
		int maxDailyDrivingTime = 9;
		
		// Setting intermediate break and daily rest related values to those of the previous label
		double dailyDrivingTime = L.dailyDrivingTime + inputdata.getTime(L.node, node); 
		double startTimeDailyRest = L.startTimeDailyRest;
		double startTimeIntermediateBreak = L.startTimeIntermediateBreak;
		double totalDistance = L.totalDistance + inputdata.getDistance(L.node, node);
		int numberDailyRests = L.numberDailyRests;
		
		// Time in the label equals max of: 1) the predecessor's time plus travel-, service- and intermediate break time to this node, 2) early time window in this node
		double arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + intermediateBreakTime, node.earlyTimeWindow); 
		
		

		
		
		
		// The driving time required on the arc
		double arcDrivingTime = inputdata.getTime(L.node, node);
		
		// The arrival time without considering the early time window in the next node
		double arrivalTimeNoWait = L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + intermediateBreakTime;
		
		// Waiting time when considering the early time window in the next node
		double waitingTime = node.earlyTimeWindow -arrivalTimeNoWait;
		
		// Computing the time left before reaching the max values of daily driving, consecutive driving, and consecutive working
		double timeLeftDriving = maxConsecutiveDrivingTime - L.consecutiveDrivingTime;
		double timeLeftWorking = maxWorkingTime - L.consecutiveWorkingTime - L.node.weight*inputdata.timeTonService;
		double timeLeftDailyDriving = maxDailyDrivingTime - L.dailyDrivingTime;
		
		// Cannot extend if a daily rest is necessary in order for the route to stay feasible
		if(timeLeftDailyDriving < arcDrivingTime) {
			return null; 
		}
		
		// Time to break is the smallest of the time left consecutive driving and working
		double timeToBreak = Math.min(timeLeftDriving, timeLeftWorking); 
		double timeDrivenBeforeFirstBreak = timeToBreak;
		
		// The start time of the intermediate break is the arrival time at the previous node plus the service time at that node and the time to break
		startTimeIntermediateBreak = L.time  + L.node.weight*inputdata.timeTonService + timeToBreak;
		
		// Consecutive driving and working time are equal to what is left driving on the arc after the intermediate break is taken
		double consecutiveDrivingTime = arcDrivingTime - timeToBreak;
		double consecutiveWorkingTime = arcDrivingTime - timeToBreak; 
		
		// If the intermediate break must be taken within the service time on the node in order to stay feasible, do not extend the label
		if (timeLeftWorking < 0) { 
			return null;
		}
		
		// If no intermediate break is necessary to stay feasible, place the daily rest at the end of the arc
		if (startTimeIntermediateBreak > L.time + (L.node.weight*inputdata.timeTonService) + arcDrivingTime) { 
			startTimeIntermediateBreak = arrivalTime - intermediateBreakTime;
			consecutiveWorkingTime = 0;
			consecutiveDrivingTime = 0;
		}
		
		// If the limit of max = 13 + 24*numberofBreaks without a daily rest is met, do not extend the label
		if(arrivalTime > 13 + 24*(numberDailyRests-1)) {  
			return null;
		}
		
		// If the limit of 13 hours driving time without a daily rest is met, do not extend the label
		if(numberDailyRests == 1 && arrivalTime > 13) {
			return null;
		}
		
		//  If the consecutive driving time is still larger than its maximum of 4.5, do not extend the label
		if (consecutiveDrivingTime > maxConsecutiveDrivingTime) {
			return null;
		}
		
		//  If the consecutive working time is still larger than its maximum of 6, do not extend the label
		if (consecutiveWorkingTime > maxWorkingTime) {
			return null;
		}
	
		// If the time is greater than the late time window of a node, do not extend the label
		if(arrivalTime > node.lateTimeWindow){
			return null;
		}
	
		// Run preprocessing on the nodes in the open nodes set
		for(int i : L.openNodes) {
			if(arrivalTime-zeroTol > preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]) {
	//			System.out.println(arrivalTime +"less than unreach: "+preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]);
	//			System.out.println(node.number+" "+(i+1));
	//			System.exit(0);
				return null;
			}
		}	
	
		if(node.type == "Depot") {
			
			// Cannot arrive at end depot without delivering every pickup that is picked up
			if(!L.openNodes.isEmpty()){
				return null;	
			}
			
			if(L.node.number == 0) {
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.totalPickupDual = L.totalPickupDual;
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			
			
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Calculating profit in the depot node
			L2.profit = L.profit 
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
			
			return L2;
		}
		
 	
		 if(node.type == "PickupNode"){
			 
			// Returns null if the node is unreachable 
			if(L.unreachablePickupNodes.contains(node.number)) {
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.node = node;
			L2.pickupDual = dualVisitedPickupsCon[node.number/2 - 1];
			L2.totalPickupDual = L.totalPickupDual + L2.pickupDual;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			L2.pickupNodesVisited.add(node.number);
			
			// Adding the weight corresponding to a pickup node if there is sufficient weight capacity on the vehicle 
			if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
				L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
			}
			else{
				return null;
			}
			
			// Adding the volume corresponding to a pickup node if there is sufficient volume capacity on the vehicle
			if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
				L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
			}
			else{
				return null;
			}
			
			// Adding all elements from the predecessor's unreachable nodes to this label's unreachable nodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Adding the node to the sets of unreachable nodes and open nodes
			L2.unreachablePickupNodes.add(node.number); 
			L2.openNodes.add(node.number);
	
			// Running preprocessing on the label and checking whether the node in unreachable
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			
			// Calculating the profit (revenue - costs) when a pickup node is visited 
				L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, L2.vehicle.nodes)))
							- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
							- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
							- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
							- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
			
			return L2;
		}
	
		
		else if(node.type == "DeliveryNode") {
			
			// Cannot visit a delivery node whose pickup node has not been visited 
			if (!L.openNodes.contains((node.number-1))){	
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.node = node;
			L2.totalPickupDual = L.totalPickupDual;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			
	
			
		
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
		
			// Remove the node's corresponding pickup node from the open nodes list when the delivery node i visited
			if (L.openNodes.contains(node.getCorrespondingNode(node, L2.vehicle.nodes).number)){
				L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, L2.vehicle.nodes).number));
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
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
			
			// Running preprocessing and checking whether the node in unreachable
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			return L2;
		}
		return null;
	}
	
	
	// Label extension with two intermediate breaks when there is no waiting time 
	public Label LabelExtensionWithTwoIntermediateBreaks(Node node, Label L, double[] dualVisitedPickupsCon) { 
		
		// Cannot return to start depot
		if(node.number == 0){
			return null;
		}
		
		// Cannot leave end depot
		if (L.node.number == 1){
			return null;
		}
		
		// Defining rule related values
		double intermediateBreakTime = 0.75;
		double maxConsecutiveDrivingTime = 4.5;
		int maxWorkingTime = 6;
		int maxDailyDrivingTime = 9;
		
		// Setting intermediate break and daily rest related values 
		double dailyDrivingTime = L.dailyDrivingTime + inputdata.getTime(L.node, node); 
		double startTimeDailyRest = L.startTimeDailyRest;
		double startTimeIntermediateBreak = L.startTimeIntermediateBreak;
		double totalDistance = L.totalDistance + inputdata.getDistance(L.node, node);
		int numberDailyRests = L.numberDailyRests;
		
		// Time in the label equals max of: 1) the predecessor's time plus travel-, service- and intermediate break time to this node, 2) early time window in this node
		double arrivalTime =  Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + 2*intermediateBreakTime, node.earlyTimeWindow); 
	//	System.out.println ("Wegiht " + L.node.weight);
	//	System.out.println ("Number " + L.node.number);
	//	System.out.println ("Wegiht2 " + node.weight);
	//	System.out.println ("Number2' " + node.number);
		// If the time is greater than the late time window of the next node, do not extend the label 
		if(arrivalTime > node.lateTimeWindow){
			return null;
		}
		
		// The driving time required on the arc
		double arcDrivingTime = inputdata.getTime(L.node, node);
		
		// Computing the time left before reaching the max values of daily driving, consecutive driving, and consecutive working
		double timeLeftDriving = maxConsecutiveDrivingTime - L.consecutiveDrivingTime;
		double timeLeftWorking = maxWorkingTime - L.consecutiveWorkingTime - L.node.weight*inputdata.timeTonService;
		double timeLeftDailyDriving = maxDailyDrivingTime - L.dailyDrivingTime;
		
		// If the maximal daily driving time is reached, the label extension cannot be executed as a daily rest is necessary
		if(timeLeftDailyDriving < arcDrivingTime) {
			return null; 
		}
		
		// Time to break is the smallest of the time left of consecutive driving and working
		double timeToBreak = Math.min(timeLeftDriving, timeLeftWorking); 
		double timeDrivenBeforeFirstBreak = timeToBreak;
		
		// The start time of the first intermediate break is the arrival time at the previous node plus the service time at that node and the time to break
		startTimeIntermediateBreak = L.time  + L.node.weight*inputdata.timeTonService + timeToBreak;
		
		// Consecutive driving and working time are equal to what is left driving on the arc after the first intermediate break is taken
		double consecutiveDrivingTime = arcDrivingTime - timeToBreak;
		double consecutiveWorkingTime = arcDrivingTime - timeToBreak;
		
		// If the intermediate break must be taken within the service time on the node in order to stay feasible, do not extend the label
		if (timeLeftWorking < 0) { 
			return null;
		}
		
		// If no intermediate break is necessary, then it is not necessary to execute this label extension
		if (startTimeIntermediateBreak > L.time + (L.node.weight*inputdata.timeTonService) + arcDrivingTime) { 
			return null;
		}
				
		// If the limit of max = 13 + 24*numberofBreaks without a daily rest is met, do not extend the label
		if(arrivalTime > 13 + 24*(numberDailyRests-1)) {  
			return null;
		}
		
		// If the limit of 13 hours driving time without a daily rest is met, do not extend the label
		if(numberDailyRests == 1 && arrivalTime > 13) {
			return null;
		}
		
		// Computing the time of the second intermediate break	 
		double timeToSecondBreak = Math.min(maxConsecutiveDrivingTime , arcDrivingTime - timeDrivenBeforeFirstBreak); 
		
		// Do not extend if the first break is already taken on the end of the arc
		if (timeToSecondBreak <= 0) { 
			return null;
		}
		
		// Computing the start time of the second intermediate break
		startTimeIntermediateBreak = Math.min(arrivalTime - intermediateBreakTime, startTimeIntermediateBreak + intermediateBreakTime + timeToSecondBreak);
		
		// If the second intermediate break must be taken before the end of the arc
		if (arcDrivingTime - timeDrivenBeforeFirstBreak > maxConsecutiveDrivingTime) {  
			consecutiveWorkingTime = arcDrivingTime - maxConsecutiveDrivingTime - timeDrivenBeforeFirstBreak;
			consecutiveDrivingTime = arcDrivingTime - maxConsecutiveDrivingTime - timeDrivenBeforeFirstBreak;
		}
		else {
			consecutiveWorkingTime = 0;
			consecutiveDrivingTime = 0;
		}
			
		// If the consecutive driving time after the second break is larger than 4.5 hours, do not extend the label 
		if (consecutiveDrivingTime > maxConsecutiveDrivingTime) {
			return null;
		}
		
		// If the consecutive working time after the second break is larger than 6 hours, do not extend the label
		if (consecutiveWorkingTime > maxWorkingTime) {
			return null;
		}
	
		// Run preprocessing on the nodes in the open nodes set
		for(int i : L.openNodes) {
			if(arrivalTime-zeroTol > preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]) {
	//			System.out.println(arrivalTime +"less than unreach: "+preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]);
	//			System.out.println(node.number+" "+(i+1));
	//			System.exit(0);
				return null;
			}
		}	
	
		
		if(node.type == "Depot") {
			
			// Cannot arrive at end depot without delivering every pickup that is picked up
			if(!L.openNodes.isEmpty()){
				return null;	
			}
			
			if(L.node.number == 0) {
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.totalPickupDual = L.totalPickupDual;
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			
			
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Calculating the profit in the depot node
			L2.profit = L.profit 
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
			
			return L2;
		}
		
 	
		if(node.type == "PickupNode"){
			
			// Returns null if the node is unreachable 
			if(L.unreachablePickupNodes.contains(node.number)) {
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.node = node;
			L2.pickupDual = dualVisitedPickupsCon[node.number/2 - 1];
			L2.totalPickupDual = L.totalPickupDual + L2.pickupDual;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			L2.pickupNodesVisited.add(node.number);
			
			// Adding the weight corresponding to a pickup node if there is sufficient weight capacity on the vehicle 
			if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
				L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
			}
			else{
				return null;
			}
			
			// Adding the volume corresponding to a pickup node if there is sufficient volume capacity on the vehicle
			if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
				L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
			}
			else{
				return null;
			}
			
			// Adding all elements from the predecessor's unreachable nodes to this label's unreachable nodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Adding the node to the sets of unreachable nodes and open nodes
			L2.unreachablePickupNodes.add(node.number); 
			L2.openNodes.add(node.number);
			
			// Running preprocessing on the label and checking whether the node in unreachable
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			
			// Calculating the profit (revenue - costs) when a pickup node is visited 
				L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, L2.vehicle.nodes)))
							- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
							- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
							- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
							- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
				
			return L2;
		}
	
		
		else if(node.type == "DeliveryNode") {
			
			// Cannot visit a delivery node whose pickup node has not been visited 
			if (!L.openNodes.contains((node.number-1))){	
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.node = node;
			L2.totalPickupDual = L.totalPickupDual;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			
		
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Remove the node's corresponding pickup node from the open nodes list when the delivery node i visited
			if (L.openNodes.contains(node.getCorrespondingNode(node, L2.vehicle.nodes).number)){
				L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, L2.vehicle.nodes).number));
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
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
			
			// Running preprocessing and checking whether the node in unreachable
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			return L2;
		}
		return null;
	}
	


	// Label extension with one intermediate break before a daily rest and then possibly another intermediate break 
	public Label LabelExtensionWithIntermediateBreakBeforeDailyRest(Node node, Label L, double[] dualVisitedPickupsCon) { 	
		
		// Cannot return to start depot
		if(node.number == 0){
			return null;
		}
		
		// Cannot leave end depot
		if (L.node.number == 1){
			return null;
		}
		
		// Defining rule related values
		double intermediateBreakTime = 0.75;
		double maxConsecutiveDrivingTime = 4.5;
		double maxWorkingTime = 6;
		int dailyRestTime = 11;
		int maxDailyDrivingTime = 9;
		
		// Setting intermediate break and daily rest related values to those of the previous label
		double startTimeDailyRest = L.startTimeDailyRest;
		double startTimeIntermediateBreak = L.startTimeIntermediateBreak;
		double consecutiveWorkingTime = L.consecutiveWorkingTime;
		double consecutiveDrivingTime = L.consecutiveDrivingTime;
		double dailyDrivingTime = L.dailyDrivingTime; 
		double totalDistance = L.totalDistance + inputdata.getDistance(L.node, node);
		int numberDailyRests = L.numberDailyRests;
		
		// Time in the label equals max of: 1) the predecessor's time plus travel-, service- and daily rest time to this node, 2) early time window in this node
		double arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime, node.earlyTimeWindow); 
		
		// Computing the waiting time when considering the early time window of the next node
		double waitingTime = node.earlyTimeWindow -  L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService  ;
		
		// The driving time required on the arc
		double arcDrivingTime = inputdata.getTime(L.node, node);
		
		// Computing the time left before reaching the max values of consecutive driving, consecutive working, daily driving and the 24-hour rule
		double timeLeftDriving = maxConsecutiveDrivingTime - L.consecutiveDrivingTime;
		double timeLeftWorking = maxWorkingTime - L.consecutiveWorkingTime - L.node.weight*inputdata.timeTonService;
		double timeTo24HourRule = (13 + 24* (numberDailyRests -1) - L.time - L.node.weight*inputdata.timeTonService );
		double timeLeftDailyDriving = (maxDailyDrivingTime - L.dailyDrivingTime);
		
		// Computing the time to when a daily rest is necessary, and when an intermediate break is necessary
		double timeToDailyRest = Math.min(timeTo24HourRule, timeLeftDailyDriving);
		double timeToBreak = Math.min(timeLeftDriving, timeLeftWorking);
		
		// Computing the time to the first intermediate break
		double drivingTimeBeforeFirstBreak = timeToBreak;

		// If it is legal to have the intermediate break come before a daily rest, place the (first) intermediate break as below
		if (timeToBreak < timeToDailyRest - intermediateBreakTime) {   
			startTimeIntermediateBreak = L.time  + L.node.weight*inputdata.timeTonService + timeToBreak;	
			// If the intermediate break must be taken within the service time, do not extend the label
			if (timeLeftWorking < 0) {  
				return null;
			}
			// Computing the time left before the daily driving time limit is reached 
			timeLeftDailyDriving = timeLeftDailyDriving - drivingTimeBeforeFirstBreak;
			// If the first break is taken at the end of the arc, no need for another break
			if (startTimeIntermediateBreak >= L.time + (L.node.weight*inputdata.timeTonService) + arcDrivingTime) { 
				return null;
			}
			// If there is no waiting time and less than 4.5 hours to drive after break, no need for a daily rest. Do not extend the label
			if (waitingTime  <= 0 && arcDrivingTime - drivingTimeBeforeFirstBreak < maxConsecutiveDrivingTime) { 
				return null;
			}
			// Computing the start time of the daily rest as the time that is reached first of the daily driving time, the consecutive driving time, the 24 hour rule, or the working time. If none are reached, place the daily rest at the end of the arc
			startTimeDailyRest = Math.min(startTimeIntermediateBreak + intermediateBreakTime + timeLeftDailyDriving - drivingTimeBeforeFirstBreak, startTimeIntermediateBreak + intermediateBreakTime + maxConsecutiveDrivingTime);		
			startTimeDailyRest = Math.min(13 + 24 * (numberDailyRests - 1), startTimeDailyRest); 
			startTimeDailyRest = Math.min(startTimeDailyRest, arrivalTime - dailyRestTime);
			startTimeDailyRest = Math.min(startTimeDailyRest, startTimeIntermediateBreak + intermediateBreakTime + maxWorkingTime);
			// Computing the arrival time including the time used for intermediate break and daily rest
			arrivalTime = Math.max(L.time + arcDrivingTime + L.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime, node.earlyTimeWindow);
			// Computing the driving time between the intermediate break and the daily rest
			double drivingTimeBetweenBreaks = startTimeDailyRest - startTimeIntermediateBreak - intermediateBreakTime;
			// Computing the consecutive driving time, the consecutive working time and the daily driving time after the daily rest
			consecutiveDrivingTime = arcDrivingTime - drivingTimeBeforeFirstBreak - drivingTimeBetweenBreaks;
			consecutiveWorkingTime = arcDrivingTime - drivingTimeBeforeFirstBreak - drivingTimeBetweenBreaks;
			dailyDrivingTime = arcDrivingTime - drivingTimeBeforeFirstBreak - drivingTimeBetweenBreaks;
			// If another intermediate break is necessary to stay feasible 
			if (consecutiveDrivingTime > maxConsecutiveDrivingTime) { 
				// Computing the arrival time with two intermediate breaks and one daily rest
				arrivalTime =  Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + 2 * intermediateBreakTime + dailyRestTime, node.earlyTimeWindow);
				// The start time of the intermediate break is 4.5 hours (max consecutive driving time) after the end of the daily rest
				startTimeIntermediateBreak = startTimeDailyRest + maxConsecutiveDrivingTime + dailyRestTime;
				// The consecutive driving and working timen, and the daily driving time when reaching the next node after the second daily rest
				consecutiveDrivingTime = arcDrivingTime - drivingTimeBetweenBreaks - maxConsecutiveDrivingTime - drivingTimeBeforeFirstBreak;
				consecutiveWorkingTime = arcDrivingTime - drivingTimeBetweenBreaks -  maxConsecutiveDrivingTime - drivingTimeBeforeFirstBreak;
				dailyDrivingTime =  arcDrivingTime -  drivingTimeBetweenBreaks - drivingTimeBeforeFirstBreak;
			}
		}	
		else {
			return null;
		}
			
		numberDailyRests = L.numberDailyRests + 1;
	
		
		// If the 24-hour rule is exceeded, do not extend the label 
		if(arrivalTime > 13 + 24*(numberDailyRests-1)) {  
			return null;
		}
	
		// If the first 24-hour rule is exceeded, do not extend the label 
		if(numberDailyRests == 1 && arrivalTime > 13) {
			return null;
		}
		
		// If the arrival time is greater than the late time window of a node, do not extend the label
		if(arrivalTime > node.lateTimeWindow){
			return null;
		}
		
		// If the limits on consecutive driving time or consecutive working time are reached, do not extend the label
		if (consecutiveDrivingTime > maxConsecutiveDrivingTime || consecutiveWorkingTime > maxWorkingTime) {
			return null;
		}
		
		// Run preprocessing on the nodes in the open nodes set
		for(int i : L.openNodes) {
			if(arrivalTime-zeroTol > preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]) {
	//			System.out.println(arrivalTime +"less than unreach: "+preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]);
	//			System.out.println(node.number+" "+(i+1));
	//			System.exit(0);
				return null;
			}
		}
	
		
		if(node.type == "Depot") {
			
			// Cannot arrive at end depot without delivering every pickup that is picked up
			if(!L.openNodes.isEmpty()){
				return null;	
			}
			
			if(L.node.number == 0) {
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.totalPickupDual = L.totalPickupDual;
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			
				
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Calculating profit in the depot node
			L2.profit = L.profit 
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
			
			return L2;
		}
		
		
		 if(node.type == "PickupNode"){
			 
			// Returns null if the node is unreachable 
			if(L.unreachablePickupNodes.contains(node.number)) {
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.node = node;
			L2.pickupDual = dualVisitedPickupsCon[node.number/2 - 1];
			L2.totalPickupDual = L.totalPickupDual + L2.pickupDual;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			L2.pickupNodesVisited.add(node.number);
			
			// Adding the weight corresponding to a pickup node if there is sufficient weight capacity on the vehicle 
			if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
				L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
			}
			else{
				return null;
			}
			
			// Adding the volume corresponding to a pickup node if there is sufficient volume capacity on the vehicle
			if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
				L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
			}
			else{
				return null;
			}
			
			// Adding all elements from the predecessor's unreachable nodes to this label's unreachable nodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Adding the node to the sets of unreachable nodes and open nodes
			L2.unreachablePickupNodes.add(node.number); 
			L2.openNodes.add(node.number);
	
			// Running preprocessing on the label and checking whether the node in unreachable
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			
			// Calculating the profit (revenue - costs) when a pickup node is visited 
				L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, L2.vehicle.nodes)))
							- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
							- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
							- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
							- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
				
			return L2;
		}
	
		
		else if(node.type == "DeliveryNode") {
			
			// Cannot visit a delivery node whose pickup node has not been visited 
			if (!L.openNodes.contains((node.number-1))){	
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.node = node;
			L2.totalPickupDual = L.totalPickupDual;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			
			
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Remove the node's corresponding pickup node from the open nodes list when the delivery node i visited
			if (L.openNodes.contains(node.getCorrespondingNode(node, L2.vehicle.nodes).number)){
				L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, L2.vehicle.nodes).number));
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
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
			
			// Running preprocessing and checking whether the node in unreachable
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			return L2;
		}
		return null;
	}
	

	// Daily rest before an intermediate break, and then possibly another daily rest
	public Label LabelExtensionWithDailyRestBeforeIntermediateBreak(Node node, Label L, double[] dualVisitedPickupsCon) { 
		
		// Cannot return to start depot
		if(node.number == 0){
			return null;
		}
		
		// Cannot leave end depot
		if (L.node.number == 1){
			return null;
		}
		
		// Defining rule related values
		double intermediateBreakTime =0.75;
		double maxConsecutiveDrivingTime = 4.5;
		int maxWorkingTime = 6;
		int dailyRestTime = 11;
		int maxDailyDrivingTime = 9;
		
		// Setting intermediate break and daily rest related values to those of the previous label
		double dailyDrivingTime = L.dailyDrivingTime; 
		double startTimeDailyRest = L.startTimeDailyRest;
		double startTimeIntermediateBreak = L.startTimeIntermediateBreak;
		double consecutiveWorkingTime = L.consecutiveWorkingTime;
		double consecutiveDrivingTime = L.consecutiveDrivingTime;	
		int numberDailyRests = L.numberDailyRests;
		double totalDistance = L.totalDistance + inputdata.getDistance(L.node, node);
		int secondDailyRest = 0;
		
		// Time in the label equals max of: 1) the predecessor's time plus travel-, service- and daily rest time to this node, 2) early time window in this node
		double arrivalTime = Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime, node.earlyTimeWindow); 
		
		// Computing the waiting time when considering the early time window in the next node
		double waitingTime = node.earlyTimeWindow -  L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService  ;
		
		// The driving time required on the arc
		double arcDrivingTime = inputdata.getTime(L.node, node);
		
		// Computing the time left before reaching the max values of consecutive driving, consecutive working, daily driving and the 24-hour rule
		double timeLeftDriving = maxConsecutiveDrivingTime - L.consecutiveDrivingTime;
		double timeLeftWorking = maxWorkingTime - L.consecutiveWorkingTime - L.node.weight*inputdata.timeTonService;
		double timeTo24HourRule = (13 + 24* (numberDailyRests -1) - L.time - L.node.weight*inputdata.timeTonService );
		double timeLeftDailyDriving = (9 - L.dailyDrivingTime);
		
		// Computing which of the rules that is most restrictive 
		double timeToDailyRest = Math.min(timeTo24HourRule, timeLeftDailyDriving);
		double timeToBreak = Math.min(timeLeftDriving, timeLeftWorking);
		double drivingTimeBeforeFirstBreak = timeToBreak;
		
		// Computing the start time of the daily rest, depending on which of the rules of daily driving, 24-hour rule, consecutive driving and consecutive working that is reached first 
		startTimeDailyRest = Math.min(L.time + L.node.weight*inputdata.timeTonService + timeLeftDailyDriving, 13 + 24 * (numberDailyRests -1));
		startTimeDailyRest = Math.min(startTimeDailyRest, L.time +  L.node.weight*inputdata.timeTonService + timeLeftDriving);
		startTimeDailyRest = Math.min(startTimeDailyRest, L.time +  L.node.weight*inputdata.timeTonService + timeLeftWorking); 
		
		// Computing the driving time before the daily rest and the remaining driving time on the arc
		timeToDailyRest = startTimeDailyRest - (L.time + L.node.weight*inputdata.timeTonService);
		double remainingDrivingTime = arcDrivingTime - timeToDailyRest;
		
		// Computing the daily driving time 
		dailyDrivingTime = arcDrivingTime - timeToDailyRest;
		double drivingTimeBeforeDailyRest = timeToDailyRest;
		
		// If the daily rest must be taken within the service time in the node, do not extend the label
		if (timeTo24HourRule < 0 && timeTo24HourRule < timeLeftWorking) { 
			return null;
		} 
		
		// If the daily rest must be taken within the service time in the node, do not extend the label 
		if (timeLeftWorking < 0 && timeLeftWorking < timeTo24HourRule) {  
			return null;
		}
		
		// Computing the start time of the intermediate break that comes after the daily rest, either where the consecutive driving time is reached, where the max driving time is reached, or at the end of the arc
		startTimeIntermediateBreak = Math.min(startTimeDailyRest + dailyRestTime + remainingDrivingTime, startTimeDailyRest + dailyRestTime + maxConsecutiveDrivingTime);
		startTimeIntermediateBreak = Math.min(startTimeIntermediateBreak, arrivalTime - intermediateBreakTime);
		
		// If the intermediate break is taken at the end of the arc, no need for another daily rest
		if (startTimeDailyRest >= L.time + (L.node.weight*inputdata.timeTonService) + arcDrivingTime) { 
			return null;
		}
		
		// If there is no waiting time and less than 4.5 hours to drive after the break, no need for another daily rest
		if (waitingTime <= 0 && arcDrivingTime < timeLeftDailyDriving + maxConsecutiveDrivingTime ) { 
			return null;
		}
		
		// If the remaining driving time is less than its maximum, then consecutive driving and working time is zero when arriving at the next node
		if(remainingDrivingTime < maxConsecutiveDrivingTime ) { 
			consecutiveDrivingTime = 0;
			consecutiveWorkingTime = 0;
		}
		// Else, the consecutive driving and working time are calculated as below
		else {
				consecutiveDrivingTime = arcDrivingTime - maxConsecutiveDrivingTime - drivingTimeBeforeDailyRest;
				consecutiveWorkingTime = arcDrivingTime - maxConsecutiveDrivingTime - drivingTimeBeforeDailyRest;
		}
		
		numberDailyRests = L.numberDailyRests + 1 ;
		
		// Checking whether a second daily rest is necessary after the daily rest and the intermediate break
		if (dailyDrivingTime > 9 || consecutiveDrivingTime > maxConsecutiveDrivingTime) { 
			// Setting the start time of the second daily rest, the new arrival time including the time for the second daily rest, and consecutive driving and working time and daily driving time
			startTimeDailyRest = Math.min(startTimeIntermediateBreak + intermediateBreakTime + maxConsecutiveDrivingTime,  13 + 24 * (numberDailyRests -1));
			arrivalTime =  Math.max(L.time+inputdata.getTime(L.node, node)+L.node.weight*inputdata.timeTonService +  intermediateBreakTime + 2 * dailyRestTime, node.earlyTimeWindow);
			consecutiveDrivingTime = arcDrivingTime -  2*maxConsecutiveDrivingTime  - drivingTimeBeforeDailyRest;
			consecutiveWorkingTime =  arcDrivingTime -  2*maxConsecutiveDrivingTime  - drivingTimeBeforeDailyRest;				
			dailyDrivingTime = arcDrivingTime -  2*maxConsecutiveDrivingTime  - drivingTimeBeforeDailyRest;
			secondDailyRest = 1;
		}
			
		numberDailyRests = L.numberDailyRests + 1 + secondDailyRest;
			
		// If the 24-hour rule is reached on the arc, do not extend the label
		if(arrivalTime > 13 + 24*(numberDailyRests-1)) {  
			return null;
		}
	
		// If the first 24-hour rule is reached on the arc, do not extend the label		 
		if(numberDailyRests == 1 && arrivalTime > 13) {
			return null;
		}
	
		// If the time is greater than the late time window of a node, no not extend the label
		if(arrivalTime > node.lateTimeWindow){
			return null;
		}
		
		// If the consecutive driving time is more than its limit of 4.5 or the consecutive working time is more than its limit of 6, do not extend the label 
		if (consecutiveDrivingTime > maxConsecutiveDrivingTime || consecutiveWorkingTime > 6) {
			return null;
		}
		
		// Run preprocessing on the nodes in the open nodes set
		for(int i : L.openNodes) {
			if(arrivalTime-zeroTol > preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]) {
	//			System.out.println(arrivalTime +"less than unreach: "+preprocess.unreachableDelNodesFromNode.get(node.number)[i+1]);
	//			System.out.println(node.number+" "+(i+1));
	//			System.exit(0);
				return null;
			}
		}
	
	
		if(node.type == "Depot") {
			
			// Cannot arrive at end depot without delivering every pickup that is picked up
			if(!L.openNodes.isEmpty()){
				return null;	
			}
			
			if(L.node.number == 0) {
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.totalPickupDual = L.totalPickupDual;
			L2.node = node;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes	
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Calculating profit in the depot node
			L2.profit = L.profit 
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
			
			return L2;
		}
		
	 	
		if(node.type == "PickupNode"){
			
			// Returns null if the node is unreachable 
			if(L.unreachablePickupNodes.contains(node.number)) {
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.node = node;
			L2.pickupDual = dualVisitedPickupsCon[node.number/2 - 1];
			L2.totalPickupDual = L.totalPickupDual + L2.pickupDual;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			L2.pickupNodesVisited.add(node.number);
			
			// Adding the weight corresponding to a pickup node if there is sufficient weight capacity on the vehicle 
			if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
				L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
			}
			else{
				return null;
			}
			
			// Adding the volume corresponding to a pickup node if there is sufficient volume capacity on the vehicle
			if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
				L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
			}
			else{
				return null;
			}
			
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Adding the node to the sets of unreachable nodes and open nodes
			L2.unreachablePickupNodes.add(node.number); 
			L2.openNodes.add(node.number);
	
			// Running preprocessing on the label and checking whether the node in unreachable
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			
			// Calculating the profit (revenue - costs) when a pickup node is visited 
				L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, L2.vehicle.nodes)))
							- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node)
							- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node)
							- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node)
							- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
				
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
			
			return L2;
		}
	
		
		else if(node.type == "DeliveryNode") {
			
			// Cannot visit a delivery node whose pickup node has not been visited 
			if (!L.openNodes.contains((node.number-1))){	
				return null;
			}
			
			// Setting values for the attributes in the extended label, L2
			Label L2 = new Label();
			L2.vehicle = L.vehicle;
			L2.vehicleDual = L.vehicleDual;
			L2.node = node;
			L2.totalPickupDual = L.totalPickupDual;
			L2.predesessor = L;
			L2.time = arrivalTime;
			L2.dailyDrivingTime = dailyDrivingTime;
			L2.startTimeDailyRest = startTimeDailyRest;
			L2.numberDailyRests = numberDailyRests;
			L2.totalDistance = totalDistance;
			L2.consecutiveDrivingTime = consecutiveDrivingTime;
			L2.startTimeIntermediateBreak = startTimeIntermediateBreak ;
			L2.consecutiveWorkingTime = consecutiveWorkingTime;
			
			L2.pickupNodesVisited = new Vector<Integer>();
			for(int i : L.pickupNodesVisited) {
				L2.pickupNodesVisited.add(i);
			}
			
		
			// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
			L2.unreachablePickupNodes = new Vector<Integer>();
			for(int i : L.unreachablePickupNodes) {
				L2.unreachablePickupNodes.add(i);
			}
			
			// Adding all elements from the predecessor's openNodes to this label's openNodes
			L2.openNodes = new Vector<Integer>();
			for(int i : L.openNodes) {
				L2.openNodes.add(i);
			}
			
			// Remove the node's corresponding pickup node from the open nodes list when the delivery node i visited
			if (L.openNodes.contains(node.getCorrespondingNode(node, L2.vehicle.nodes).number)){
				L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, L2.vehicle.nodes).number));
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
			
			// Calculating the reduced cost of the label
			L2.reducedCost = L2.profit - L2.totalPickupDual - L2.vehicleDual;
			
			// Running preprocessing and checking whether the node in unreachable
			for(Node pickup: pickupNodes) {
				if(!L2.unreachablePickupNodes.contains(pickup.number)) {
					if(preprocess.unreachableNodesFromNode.get(node.number)[pickup.number]+zeroTol<L2.time)
					L2.unreachablePickupNodes.add(pickup.number);
				}
			}
			return L2;	
		}
		return null;
	}
	
	
	
	public Vector<Label> BuildPaths(Vehicle vehicle, double[] dualVisitedPickupsCon, double[] dualOneVisitCon, BBNode bbNode) {
		preprocess = new Preprocessing(pickupNodes, deliveryNodes, vehicles, inputdata, nodesWithoutDepot, vehicle);
		preprocess.unreachableNodeCombination();
		preprocess.unreachableDeliveryNode();
		preprocess.unreachableDeliveryPairs();
		
		// Creating the list of non-dominated labels
		Vector<Label> list = new Vector<Label>();   
		Vector<Label> labelsFound = new Vector<Label>();
		// Initializing label
		Label L = new Label();
	//	L.bestLabelNumber = 0;
		L.vehicle = vehicle;
		L.node = vehicle.startDepot;
		L.time = 0;
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
		L.pickupNodesVisited = new Vector<Integer>();
		
	//	for(int k = 0; k < vehicles.size(); k++) {
	//		System.out.println("Vehicle:" +vehicle.number);
	//		System.out.println(dualOneVisitCon.get(k));
	//	}
		L.vehicleDual = dualOneVisitCon[vehicle.number];
		L.pickupDual = 0;
		L.reducedCost = L.profit -L.totalPickupDual - L.vehicleDual;
	
		L.totalPickupDual = 0;
		// Creating lists unprocessed labels at node i, and processed labels at node i
		ArrayList<Vector<Label>> unprocessedAtNode = new ArrayList<Vector<Label>>();
		ArrayList<Vector<Label>> processedAtNode = new ArrayList<Vector<Label>>();
		// Adding nodes to the processed and unprocessed lists
		for(int i = 0; i < L.vehicle.nodes.size(); i++) {
			Vector<Label> processed = new Vector<Label>();
			processedAtNode.add(i, processed);
			Vector<Label> unprocessed = new Vector<Label>();
			unprocessedAtNode.add(i, unprocessed);
		}
		// Organizing the list of unprocessed labels such that those with smallest time is selected first
		PriorityQueue<Label> unprocessedQueue = new PriorityQueue<Label>(5, new UnprocessedComparator()); 
		// Adding L to the set of unprocessed labels
		unprocessedQueue.add(L);
		int counter = 0;
		//Going through all unprocessed labels
		int labelsFoundCounter = 0;
		while(!unprocessedQueue.isEmpty() ) { 
		//	System.out.println(labelsFoundCounter);
		//	System.out.println(unprocessedQueue.size());
			Label label = unprocessedQueue.remove();
		//	System.out.println("Remove from unprocessed "+label.toString());
			counter++;
			// Print current label every 1000 labels
			//if(counter%1000 == 0) {
				//System.out.println(counter+" "+label.toString());
				//System.out.println("number of unprocessed labels: "+unprocessedQueue.size());
			//}
			
			// Going through all pickup nodes 
			for(Node pickup:pickupNodes) { 
				if(bbNode.branchingMatrix[vehicle.number][(pickup.number/2) - 1] != -1) {
					
				
				double arcDrivingTime = inputdata.getTime(label.node,  pickup);
				double dailyDrivingTime = label.dailyDrivingTime;
				int maxDailyDrivingTime = 9;
				
				// Only extend labels without daily rest if the arc driving time plus the daily driving time is less than 9 (no daily rest necessary)
				if (arcDrivingTime + dailyDrivingTime <= maxDailyDrivingTime){
					
					// Run label extension without daily rest or intermediate break and check dominance
					Label newLabel = LabelExtension(pickup, label, dualVisitedPickupsCon);
					if(newLabel!=null) {
						//System.out.println(newLabel.toString());
						if(checkdominance(newLabel, unprocessedQueue, unprocessedAtNode.get(newLabel.node.number), processedAtNode.get(newLabel.node.number), bbNode)) {
							unprocessedQueue.add(newLabel); 
							unprocessedAtNode.get(newLabel.node.number).add(newLabel);
							
						}
					}
					
					// Run label extension with intermediate break and check dominance
					Label newLabel2 = LabelExtensionWithIntermediateBreak(pickup, label, dualVisitedPickupsCon);
					
					if(newLabel2!=null) {
						if(checkdominance(newLabel2, unprocessedQueue, unprocessedAtNode.get(newLabel2.node.number), processedAtNode.get(newLabel2.node.number), bbNode)) {
							unprocessedQueue.add(newLabel2); 
							unprocessedAtNode.get(newLabel2.node.number).add(newLabel2);
							
						}
					}
					
					// Run label extension with two intermediate breaks and check dominance
					Label newLabel3 = LabelExtensionWithTwoIntermediateBreaks(pickup, label, dualVisitedPickupsCon);
					
					if(newLabel3!=null) {
						if(checkdominance(newLabel3, unprocessedQueue, unprocessedAtNode.get(newLabel3.node.number), processedAtNode.get(newLabel3.node.number), bbNode)) {
							unprocessedQueue.add(newLabel3); 
							unprocessedAtNode.get(newLabel3.node.number).add(newLabel3);
						}
					}
				
				}
				
				// Run label extension with daily rest and check dominance
				Label newLabel4 = LabelExtensionWithDailyRest(pickup, label, dualVisitedPickupsCon);
				
				
				if(newLabel4!=null) {
					
					if(checkdominance(newLabel4, unprocessedQueue, unprocessedAtNode.get(newLabel4.node.number), processedAtNode.get(newLabel4.node.number), bbNode)) {
						unprocessedQueue.add(newLabel4); 
						unprocessedAtNode.get(newLabel4.node.number).add(newLabel4);
	
					}
				}
				
				
				double intermediateBreakTime = 0.75;
				double maxDrivingTime = 4.5;
				int dailyRestTime = 11;
				double waitingTime = pickup.earlyTimeWindow - (label.time + inputdata.getTime(label.node,  pickup) + label.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime);
				
				// If the waiting time is above zero or the arc driving time is above its maximum of 4.5, execute the following label extensions
				if (waitingTime > 0 || arcDrivingTime > maxDrivingTime) {
					
					// Run label extension with intermediate break and then daily rest and check dominance
					Label newLabel5 = LabelExtensionWithIntermediateBreakBeforeDailyRest(pickup, label, dualVisitedPickupsCon);
					
						if(newLabel5!=null) {
							if(checkdominance(newLabel5, unprocessedQueue, unprocessedAtNode.get(newLabel5.node.number), processedAtNode.get(newLabel5.node.number), bbNode)) {
								unprocessedQueue.add(newLabel5); 
								unprocessedAtNode.get(newLabel5.node.number).add(newLabel5);
							}
						}			
					
					
					// Run label extension with daily rest and then intermediate break and check dominance
					Label newLabel6 = LabelExtensionWithDailyRestBeforeIntermediateBreak(pickup, label, dualVisitedPickupsCon);
					
						if(newLabel6!=null) {
							if(checkdominance(newLabel6, unprocessedQueue, unprocessedAtNode.get(newLabel6.node.number), processedAtNode.get(newLabel6.node.number), bbNode)) {
								unprocessedQueue.add(newLabel6); 
								unprocessedAtNode.get(newLabel6.node.number).add(newLabel6);
							}
						}			
				}}
			}	
			
			// Going through all nodes in the open nodes set (visited pickup nodes), and get their corresponding delivery node
			for(int i : label.openNodes) { 
				
				Node node = L.vehicle.nodes.get(i+1); 
				
				double arcDrivingTime = inputdata.getTime(label.node, node);
				double intermediateBreakTime = 0.75;
				double maxDrivingTime = 4.5;
				int dailyRestTime = 11;
				int maxDailyDrivingTime = 9;
				double waitingTime = node.earlyTimeWindow - (label.time + inputdata.getTime(label.node,  node) + label.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime);
				double dailyDrivingTime = label.dailyDrivingTime;
				
				// Only extend labels without daily rest if the arc driving time plus the daily driving time is less than 9 (no daily rest necessary)
				if (arcDrivingTime + dailyDrivingTime < maxDailyDrivingTime) {
				//	System.out.println("TYPE"+L.vehicle.nodes.get(i+1).type);
					// Run label extension without daily rest or intermediate break and check dominance
					Label newLabel = LabelExtension(L.vehicle.nodes.get(i+1), label, dualVisitedPickupsCon);
					
					if(newLabel!=null) {
						//System.out.println(newLabel.toString());
						if(checkdominance(newLabel, unprocessedQueue, unprocessedAtNode.get(newLabel.node.number), processedAtNode.get(newLabel.node.number), bbNode)) {
							unprocessedQueue.add(newLabel); 
							unprocessedAtNode.get(newLabel.node.number).add(newLabel);
						}
					}
					
					// Run label extension with intermediate break and check dominance
					Label newLabel2 = LabelExtensionWithIntermediateBreak(L.vehicle.nodes.get(i+1), label, dualVisitedPickupsCon);
					
					if(newLabel2!=null) {
						//System.out.println(newLabel2.toString());
						if(checkdominance(newLabel2, unprocessedQueue, unprocessedAtNode.get(newLabel2.node.number), processedAtNode.get(newLabel2.node.number), bbNode)) {
							unprocessedQueue.add(newLabel2); 
							unprocessedAtNode.get(newLabel2.node.number).add(newLabel2);
						}
					}
					
					// Run label extension without two intermediate breaks and check dominance
					Label newLabel3 = LabelExtensionWithTwoIntermediateBreaks(L.vehicle.nodes.get(i+1), label, dualVisitedPickupsCon);
					
					if(newLabel3!=null) {
					//	System.out.println(newLabel3.toString());
						if(checkdominance(newLabel3, unprocessedQueue, unprocessedAtNode.get(newLabel3.node.number), processedAtNode.get(newLabel3.node.number), bbNode)) {
							unprocessedQueue.add(newLabel3); 
							unprocessedAtNode.get(newLabel3.node.number).add(newLabel3);
						}
					}
				}
				
				
				// Run label extension with daily rest and check dominance
				Label newLabel4 = LabelExtensionWithDailyRest(L.vehicle.nodes.get(i+1), label, dualVisitedPickupsCon);
				
				if(newLabel4!=null) {
				//	System.out.println(newLabel4.toString());
					if(checkdominance(newLabel4, unprocessedQueue, unprocessedAtNode.get(newLabel4.node.number), processedAtNode.get(newLabel4.node.number), bbNode)) {
						unprocessedQueue.add(newLabel4); 
						unprocessedAtNode.get(newLabel4.node.number).add(newLabel4);
					}
				}
				

				// If the waiting time is above zero or the arc driving time is above its maximum of 4.5, execute the following label extensions
				if (waitingTime > 0 || arcDrivingTime > maxDrivingTime) {
				
					// Run label extension with intermediate break and then daily rest and check dominance
					Label newLabel5 = LabelExtensionWithIntermediateBreakBeforeDailyRest(L.vehicle.nodes.get(i+1), label, dualVisitedPickupsCon);
					
					if(newLabel5!=null) {
					//	System.out.println(newLabel5.toString());
						if(checkdominance(newLabel5, unprocessedQueue, unprocessedAtNode.get(newLabel5.node.number), processedAtNode.get(newLabel5.node.number), bbNode)) {
							unprocessedQueue.add(newLabel5); 
							unprocessedAtNode.get(newLabel5.node.number).add(newLabel5);
						}
					}
					
					// Run label extension with daily rest and then intermediate break and check dominance	
					Label newLabel6 = LabelExtensionWithDailyRestBeforeIntermediateBreak(L.vehicle.nodes.get(i+1), label, dualVisitedPickupsCon);
					
					if(newLabel6!=null) {
					//	System.out.println(newLabel6.toString());
						if(checkdominance(newLabel6, unprocessedQueue, unprocessedAtNode.get(newLabel6.node.number), processedAtNode.get(newLabel6.node.number), bbNode)) {
							unprocessedQueue.add(newLabel6); 
							unprocessedAtNode.get(newLabel6.node.number).add(newLabel6);
						}
					}
				}
			}
		
			// Extending labels to the end depot
			Node node = L.vehicle.nodes.get(1);
			
			//System.out.println(L.vehicle.number);
			//for(Node i : L.vehicle.nodes) {
			//	System.out.println(i.lateTimeWindow);
			//}
			double arcDrivingTime = inputdata.getTime(label.node, node);
			double dailyDrivingTime = label.dailyDrivingTime;
			int maxDailyDrivingTime = 9;
			
			// Only extend labels without daily rest if the arc driving time plus the daily driving time is less than 9 (no daily rest necessary)
			if (arcDrivingTime + dailyDrivingTime < maxDailyDrivingTime) {
			
				// Run label extension without daily rest or intermediate break and check dominance
				Label newLabel = LabelExtension(L.vehicle.nodes.get(1), label, dualVisitedPickupsCon); 
		//		System.out.println(L.vehicle.nodes.get(1).type);
				
				if(newLabel!=null) {
					
				//	System.out.println(newLabel);
					if(checkdominance(newLabel, unprocessedQueue, unprocessedAtNode.get(newLabel.node.number), processedAtNode.get(newLabel.node.number), bbNode)) {
						list.add(newLabel);
						if(newLabel.reducedCost > 0.0001) {
							labelsFound.add(newLabel);
							labelsFoundCounter++;
						}
					}
				}
				
				
				// Run label extension with intermediate break and check dominance
				Label newLabel2 = LabelExtensionWithIntermediateBreak(L.vehicle.nodes.get(1), label, dualVisitedPickupsCon);
				
				if(newLabel2!=null) {
					//System.out.println(newLabel2);
					if(checkdominance(newLabel2, unprocessedQueue, unprocessedAtNode.get(newLabel2.node.number), processedAtNode.get(newLabel2.node.number), bbNode)) {
						list.add(newLabel2);
						if(newLabel2.reducedCost > 0.0001) {
							labelsFound.add(newLabel2);
							labelsFoundCounter++;
						}
					}
				}
				
				// Run label extension with two intermediate breaks and check dominance
				Label newLabel3 = LabelExtensionWithTwoIntermediateBreaks(L.vehicle.nodes.get(1), label, dualVisitedPickupsCon);
				
				if(newLabel3!=null) {
				//	System.out.println(newLabel3);
					if(checkdominance(newLabel3, unprocessedQueue, unprocessedAtNode.get(newLabel3.node.number), processedAtNode.get(newLabel3.node.number), bbNode)) {
						list.add(newLabel3);
						if(newLabel3.reducedCost > 0.0001) {
							labelsFound.add(newLabel3);
							labelsFoundCounter++;
						}
					}
				}
			}
			
			// Run label extension with daily rest and check dominance
			Label newLabel4 = LabelExtensionWithDailyRest(L.vehicle.nodes.get(1), label, dualVisitedPickupsCon);
			
			if(newLabel4!=null) {
				//System.out.println(newLabel4);
				if(checkdominance(newLabel4, unprocessedQueue, unprocessedAtNode.get(newLabel4.node.number), processedAtNode.get(newLabel4.node.number), bbNode)) {
					list.add(newLabel4);
					if(newLabel4.reducedCost > 0.0001) {
						labelsFound.add(newLabel4);
						labelsFoundCounter++;
					}
				}
			}
			
			
			double intermediateBreakTime = 0.75;
			double maxDrivingTime = 4.5;
			int dailyRestTime = 11;
			double waitingTime = node.earlyTimeWindow - (label.time + inputdata.getTime(label.node,  node) + label.node.weight*inputdata.timeTonService + intermediateBreakTime + dailyRestTime);
			
			// If the waiting time is above zero or the arc driving time is above its maximum of 4.5, execute the following label extensions
			if (waitingTime > 0 || arcDrivingTime > maxDrivingTime) {
			
				// Run label extension with intermediate break and then daily rest and check dominance
				Label newLabel5 = LabelExtensionWithIntermediateBreakBeforeDailyRest(L.vehicle.nodes.get(1), label, dualVisitedPickupsCon);
				
				if(newLabel5!=null) {
					
					if(checkdominance(newLabel5, unprocessedQueue, unprocessedAtNode.get(newLabel5.node.number), processedAtNode.get(newLabel5.node.number), bbNode)) {
						list.add(newLabel5);
						if(newLabel5.reducedCost > 0.0001) {
							labelsFound.add(newLabel5);
							labelsFoundCounter++;
						}
					}
				}
				
				// Run label extension with daily rest and then intermediate break and check dominance	
				Label newLabel6 = LabelExtensionWithDailyRestBeforeIntermediateBreak(L.vehicle.nodes.get(1), label, dualVisitedPickupsCon);
				
				if(newLabel6!=null) {
					
					if(checkdominance(newLabel6, unprocessedQueue, unprocessedAtNode.get(newLabel6.node.number), processedAtNode.get(newLabel6.node.number), bbNode)) {
						list.add(newLabel6);
						if(newLabel6.reducedCost > 0.0001) {
							labelsFound.add(newLabel6);
							labelsFoundCounter++;
						}
					}
				}
			}
			// The label removed from unprocessed list is added to the processed list
			processedAtNode.get(label.node.number).add(label); 
			
			//if(labelsFoundCounter >= 5) {
			//	break;
			//}
		}
		
		//	System.out.println("Number of paths:" + processed.size());
		System.out.println("number of non-dominated paths: "+list.size());
		pw.println("number of non-dominated paths: "+list.size());
		numberOfPaths += list.size();
		System.out.println(numberOfPaths);
		System.out.println("number of dominated labels: "+numberOfDominatedLabels);
		pw.println("number of dominated labels: "+numberOfDominatedLabels);
		System.out.println("The best label is:");
		pw.println ("The best label is: ");
//		System.out.println(findBestLabel(list).toString());
		//pw.println(findBestLabel(list).toString());
		//for(Label i : list) {
		//System.out.println(i.toString());
		//}
		
	//	return list;
		return labelsFound;
	}
	
	// Checks if L1 dominates L2
	private boolean dominateLabel(Label L1, Label L2, BBNode bbNode) { 
		
		if (L1.node.number != L2.node.number) {
			return false;
		}
		if (L1.reducedCost+zeroTol<L2.reducedCost) {
			return false;
		}
		if (L1.time-zeroTol>L2.time) {
			return false;
		}
		for(int pickup : bbNode.branchingMatrix[L1.vehicle.number]) {
			int pickupNumber = 2;
			if(pickup == 1) {
				if(!L1.pickupNodesVisited.contains(pickupNumber) && L2.pickupNodesVisited.contains(pickupNumber)) {
					return false;
				}
			}
			pickupNumber += 2;
		}
		
		if( L1.startTimeDailyRest >= L2.startTimeDailyRest) {	
			if ( L1.dailyDrivingTime <= L2.dailyDrivingTime && L1.numberDailyRests >= L2.numberDailyRests ) {
				if ( L1.consecutiveDrivingTime <= L2.consecutiveDrivingTime) {
						if (L1.consecutiveWorkingTime <= L2.consecutiveWorkingTime) {
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
							return true;	
						}	
					else return false; 	
				}
				else return false; 	
			}
			else return false; 	
		}
		else return false; 
	}
	
	
	
	// Updates the processed and unprocessed lists according to the dominated labels
	private boolean checkdominance(Label newLabel, PriorityQueue<Label> unprocessedQueue, Vector<Label> unprocessed, Vector<Label> processed, BBNode bbNode) {
		Vector<Label> remove = new Vector<Label>();
		
		for(Label oldLabel : unprocessed) {
			if(dominateLabel(oldLabel, newLabel, bbNode)) {
				numberOfDominatedLabels++;
				unprocessedQueue.removeAll(remove);
				unprocessed.removeAll(remove);
				return false;
			}
			else if(dominateLabel(newLabel,oldLabel, bbNode)) {
				remove.add(oldLabel);
				numberOfDominatedLabels++;
			}
		}
		unprocessedQueue.removeAll(remove);
		unprocessed.removeAll(remove);
		
		remove = new Vector<Label>();
		for(Label oldLabel : processed) {
			if(dominateLabel(oldLabel, newLabel, bbNode)) {
				processed.removeAll(remove);
				numberOfDominatedLabels++;
				return false;
			}
			else if(dominateLabel(newLabel,oldLabel, bbNode)) {
				numberOfDominatedLabels++;
				remove.add(oldLabel);
			}
		}
		processed.removeAll(remove);
		
		return true;
	}
/*	
	// Finds the non-dominated label with the best profit and returns it as the best solution
	public Label findBestLabel(Vector<Label> list, BBNode bbNode, Hashtable<Integer,Label> pathList, Vehicle v) throws NullPointerException {
		double currentBestRedCost =  0.0001;
		Label bestLabel = null;
	//	ArrayList<Label> bestLabels = new ArrayList<Label>();
	//	System.out.println(list);
		for(Label i : list) {
			
			if(i.reducedCost > currentBestRedCost) {
				for(int pickup : bbNode.branchingMatrix[i.vehicle.number]) {
					int pickupNumber = 2;
					if(pickup == 1 && !i.pickupNodesVisited.contains(pickupNumber)) {
						break;
					}
					else {
						currentBestRedCost = i.reducedCost;
						bestLabel = i;
					}
					pickupNumber += 2;
				}
				
			}
		}
		
		
				
				//System.out.println(i.reducedCost);
				//currentBestRedCost = i.reducedCost;
				//bestLabel = i;
			
		
		
		if (bestLabel == null) {
//			throw new NullPointerException ("No feasible solution");	
			return null;
		}
	
		//for(Label i : bestLabels) {
			bestLabel.path = new Vector<Node>();
		//	i.pickupNodesVisited = new Vector<Integer>();	
			Label temp = bestLabel.predesessor;
			while(temp!=null) {
			//	System.out.println(temp.toString());
				//pw.println(temp.toString());
				bestLabel.path.add(temp.node);
			//	if(temp.node.type == "PickupNode") {
			//		i.pickupNodesVisited.add(temp.node.number);
			//	}
			temp=temp.predesessor;
		}
			
	//	for(Label label : pathList
		//	System.out.println(i.toString());
		
		//Route route = new Route();
		//routes.add(route);
		
		//route.vehicle = bestLabel.vehicle;
		
		//route.profit = bestLabel.profit;
		//reducedCost = bestLabel.reducedCost;
		
		
		
		
		
	//	bestLabel.bestLabelNumber = bestLabelNumber;
		 //bestLabelNumber += 1;
		
	
		//pw.println(bestLabel.toString());
		
		
		return bestLabel;
	}*/
	
	// TODO: velg ut feks 20 beste som sendes til masterproblem - for disse legg til path og sjekk path mot pathList at ingen har lik path som allerede finnes (list1.equals(list2))
	
	
	// Finds the non-dominated label with the best profit and returns it as the best solution
		public ArrayList<Label> findBestLabel(Vector<Label> labelsFound, BBNode bbNode,  Hashtable<Integer,Label> pathList, Vehicle v) throws NullPointerException {
			double currentBestRedCost =  0.0001;
			System.out.println(labelsFound.size());
			//Label bestLabel = null;
			PriorityQueue<Label> bestLabelQueue = new PriorityQueue<Label>(1000, new BestLabelComparator()); 
			ArrayList<Label> bestLabels = new ArrayList<Label>();
			//System.out.println(list);
			for(Label i : labelsFound) {
			//	if(i.reducedCost > currentBestRedCost) {
					boolean addLabel = true;
					int pickupNumber = 2;
					for(int pickup : bbNode.branchingMatrix[i.vehicle.number]) {
						if(pickup == 1 && !i.pickupNodesVisited.contains(pickupNumber)) {
							addLabel = false;
							break;
						}
						//else {
							//if(i.reducedCost > currentBestRedCost) {
							//}
						//	bestLabels.add(i);
						//}
						pickupNumber += 2;
					}
					if (addLabel == true) {
						bestLabelQueue.add(i);
					}
					
					//System.out.println(i.reducedCost);
					//currentBestRedCost = i.reducedCost;
					//bestLabel = i;
		//		}
			}System.out.println(bestLabelQueue.size());
		//	for(int i = 0 ; i < bestLabelQueue.size(); i ++) {
		//		pw.println(i);
		//		pw.println(bestLabelQueue.remove().reducedCost);
		//	}
			
			
			while(bestLabels.size()<Math.min(100, bestLabelQueue.size())) {
				Label currentBestLabel = bestLabelQueue.remove();
			//	System.out.println(bestLabelQueue.size());
				currentBestLabel.path = new Vector<Node>();
				Label temp = currentBestLabel.predesessor;
				while(temp!=null) {
					//	System.out.println(temp.toString());
						//pw.println(temp.toString());
						currentBestLabel.path.add(temp.node);
					//	if(temp.node.type == "PickupNode") {
					//		i.pickupNodesVisited.add(temp.node.number);
					//	}
					temp=temp.predesessor;
				}
			//	int number = 0;
				int routeNumber;
				boolean routeAlreadyExisting = false;
				for(int i = 0; i < v.vehicleRoutes.size(); i++) {
					routeNumber = vehicles.get(v.number).vehicleRoutes.get(i);
					if(pathList.get(routeNumber).path != null && pathList.get(routeNumber).path.equals(currentBestLabel.path) && pathList.get(routeNumber).time == currentBestLabel.time && pathList.get(routeNumber).startTimeIntermediateBreak == currentBestLabel.startTimeIntermediateBreak){
						routeAlreadyExisting = true;
			//			System.out.println("HER er vi");
						break;
					}
				}
				if(! routeAlreadyExisting) {
					for(Label label : bestLabels) {
						if(label.path.equals(currentBestLabel.path) && label.reducedCost == currentBestLabel.reducedCost && label.time == currentBestLabel.time) {
							routeAlreadyExisting = true;
							break;
						}
					}
					if(! routeAlreadyExisting ) {
						bestLabels.add(currentBestLabel);
				//		pw.println("Current best " + currentBestLabel.toString());
					}
				}
				
				//	number ++;
			//	int pickupNumber = 2;
				
				//for (int pickup : bbNode.branchingMatrix[v.number]) {
				//	for(Node pickupNumber : pickupNodes) {
						//System.out.println(pathList.get(4).profit);
					//	if (pathList.get(routeNumber).pickupNodesVisited != null && pathList.get(routeNumber).pickupNodesVisited.contains(pickupNumber.number) && pickup == -1) {
				
			
			//for(int i = 0; i < Math.min(20, bestLabelQueue.size()); i++) {
				
				
			}
			
			if (bestLabels.isEmpty()) {
//				throw new NullPointerException ("No feasible solution");	
				return null;
			}
		
		/*	for(Label i : bestLabels) {
				i.path = new Vector<Node>();
			//	i.pickupNodesVisited = new Vector<Integer>();	
				Label temp = i.predesessor;
				while(temp!=null) {
				//	System.out.println(temp.toString());
					//pw.println(temp.toString());
					i.path.add(temp.node);
				//	if(temp.node.type == "PickupNode") {
				//		i.pickupNodesVisited.add(temp.node.number);
				//	}
				temp=temp.predesessor;
			}
			//	System.out.println(i.toString());
			
			//Route route = new Route();
			//routes.add(route);
			
			//route.vehicle = bestLabel.vehicle;
			
			//route.profit = bestLabel.profit;
			//reducedCost = bestLabel.reducedCost;
			
			} */
			
			
			
		//	bestLabel.bestLabelNumber = bestLabelNumber;
			 //bestLabelNumber += 1;
			
		
			//pw.println(bestLabel.toString());
			
			
			return bestLabels;
		}
	
}
