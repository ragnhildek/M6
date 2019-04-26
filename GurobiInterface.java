import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;
import java.util.Vector;
import gurobi.GRB.DoubleAttr;
import gurobi.GRB.IntAttr;
import gurobi.*;

	public class GurobiInterface {
		
		private Hashtable<Integer, Label> pathList;
		public Hashtable<Integer, Double> MPsolutionVars;
		public ArrayList<Integer> routesUtilized;
		
		// Creating Gurobi environment
	    GRBEnv    env   = new GRBEnv("mip2.log");
	    GRBModel  model;
	    
	    // Creating Gurobi variables
	    private Vector<GRBVar> variables;
//	    private GRBVar[][] lambdaVars;
		public ArrayList<GRBVar>[] lambdaVars;
	    
	    // Creating Gurobi constraints
		public GRBConstr[] visitedPickupsCon;
		public GRBConstr[] oneVisitCon; 
		
		// Creating Gurobi objective function
		public GRBLinExpr objective;
		
		// Lists of dual variables
		
		public double zeroTol = 0.0001;
	    public InstanceData inputdata;
		public double profit = 0;
		public Vector<Node> pickupNodes;
		public Vector<Node> deliveryNodes;
		public Vector<Node> nodesWithoutDepot;
	//	public Vector<Node> nodes;
	//	public Vector<Node> depot;
		public PrintWriter pw;
		public Vector<Node> path;
		//public Vector<Route> routes;
		public Vector<Vehicle> vehicles;
		public PathBuilder builder;
		//public Route route;
		public int numberOfRoutes = 0;
		
		//public Vector<Vector<Vector<Integer>>> visitedPickupsByVehicleOnRoute;
//		int[][][] visitedPickupsByVehicleOnRoute;
	
		 
		//private Hashtable<Integer, Route> pathList;
		
		public GRBLinExpr visitedPickupsLeftHand[];
		public GRBLinExpr oneVisitLeftHand[];
		
		public GRBColumn col;
		public GRBColumn col2;
		
		public GurobiInterface(InstanceData inputdata, Vector<Node> deliveryNodes, Vector<Node> pickupNodes, Vector<Node> nodesWithoutDepot, Vector<Vehicle> vehicles, Vector<Double> dualVisitedPickupsCon, Vector<Double> dualOneVisitCon, PrintWriter pw) throws Exception {
			env.set(GRB.IntParam.Presolve, 0);
			env.set(GRB.DoubleParam.OptimalityTol, 0.000000001);
			env.set(GRB.DoubleParam.FeasibilityTol, 0.000000001);
			
			model = new GRBModel(env);
			
			this.vehicles = vehicles; 
			//this.builder = new PathBuilder(pickupNodes, deliveryNodes, nodes, depot, inputdata, pw, vehicles);
			//this.path = route.path;
			//this.profit = label.profit;
			this.inputdata = inputdata;
			this.pickupNodes = pickupNodes;
			this.deliveryNodes = deliveryNodes;
			this.nodesWithoutDepot = nodesWithoutDepot;
		//	this.depot = depot;
		//	this.nodes = nodes;
			this.pw = pw;
			this.vehicles = vehicles;
			this.pathList = new Hashtable<Integer, Label>();
			//solveProblem();
		}
		
		public void buildProblem() throws Exception {
			this.objective = new GRBLinExpr();
			
			this.variables = new Vector<GRBVar>();
//			this.lambdaVars = new GRBVar[vehicles.size()][100];
			this.lambdaVars = new ArrayList[vehicles.size()];
			
			
			this.visitedPickupsCon = new GRBConstr[pickupNodes.size()];
			this.oneVisitCon = new GRBConstr[vehicles.size()];
			
			this.oneVisitLeftHand = new GRBLinExpr[vehicles.size()];
			this.visitedPickupsLeftHand = new GRBLinExpr[pickupNodes.size()];
			
		
			
		//	this.col = new GRBColumn();
			
			//Label firstLabel = new Label();
			//firstLabel.profit = 0;
			Node firstNode = new Node(0);
			Label firstLabel = new Label();
			firstLabel.profit = 0;
			//firstLabel.path.add(firstNode);
		
		
			
//			this.visitedPickupsByVehicleOnRoute = new int[100][vehicles.size()][pickupNodes.size()];
			

	
			
			
			for(int k = 0; k < vehicles.size(); k++) {
				this.lambdaVars[k] = new ArrayList<GRBVar>();
				this.vehicles.get(k).vehicleRoutes = new Vector<Integer>();
				
				numberOfRoutes += 1;
				//System.out.println(vehicles.get(k).startDepot.number);
				//System.out.println(vehicles.get(k).vehicleRoutes);
				vehicles.get(k).vehicleRoutes.add(numberOfRoutes);
				for (int r : vehicles.get(k).vehicleRoutes) {
					//System.out.println("vehicleroutes" + vehicles.get(k).vehicleRoutes);
					GRBVar temp = model.addVar(0, GRB.INFINITY, firstLabel.profit, GRB.CONTINUOUS, col, "lambda_"+k+"_"+r);
					this.lambdaVars[k].add(temp);
					
					pathList.put(numberOfRoutes-1, firstLabel);
					this.objective.addTerm(firstLabel.profit, temp);
//					for(int k = 0; k < vehicles.size(); k++) {
						GRBLinExpr temp2 = new GRBLinExpr();
						temp2.addTerm(1, temp);
						this.oneVisitCon[k] = model.addConstr(temp2, GRB.EQUAL, 1, "oneVisitCon"+k);		// skal egentlig være Equal 
				
				}
			}
			model.setObjective(objective, GRB.MAXIMIZE);
			
			model.update();
			
			// visited pickups constraint 
			
			for(int i = 0; i < pickupNodes.size(); i++) {
				GRBLinExpr temp = new GRBLinExpr();
//				for(int k = 0; k < vehicles.size(); k++) {
//					for(int r : vehicles.get(k).vehicleRoutes) {
//					//	System.out.println(r);
//						temp.addTerm(visitedPickupsByVehicleOnRoute[r][k][i], lambdaVars[k][r]);
////						System.out.println(visitedPickupsByVehicleOnRoute[r][k][i]);
//					
//					}
//				}
				this.visitedPickupsCon[i] = model.addConstr(temp, GRB.LESS_EQUAL,1,"visitedPickupCon"+i);	
			}
			
			model.update();
			
			
			// one visit per vehicle constraint 
			
//			for(int k = 0; k < vehicles.size(); k++) {
//				GRBLinExpr temp = new GRBLinExpr();
//				for (int r : vehicles.get(k).vehicleRoutes) {
//					temp.addTerm(1, lambdaVars[k][r]);
//
//				}
//				this.oneVisitCon[k] = model.addConstr(temp, GRB.EQUAL, 1, "oneVisitCon"+k);		// skal egentlig være Equal 
//			}
//			
			model.update();

		}
		
		
		
		public void addRoute(Label l) throws Exception{
			
			//col2 = new GRBColumn();
			
			GRBVar tempVar = model.addVar(0, GRB.INFINITY, l.profit, GRB.CONTINUOUS,  "lambda_"+l.vehicle.number + "_" + numberOfRoutes);
			lambdaVars[l.vehicle.number].add(tempVar);
			this.objective.addTerm(l.profit, tempVar);
			
			
			
			for(int i = 0; i < pickupNodes.size(); i++) {
			
				if(l.pickupNodesVisited.contains(pickupNodes.get(i).number)) {
				//	System.out.println("PICKUP" + i);
					model.chgCoeff(visitedPickupsCon[i], tempVar, 1);
//					System.out.println(visitedPickupsByVehicleOnRoute[l.vehicle.vehicleRoutes.lastElement()][l.vehicle.number][i]);
				}	
			}
				model.chgCoeff(oneVisitCon[l.vehicle.number], tempVar, 1);
			//	System.out.println("Variables"+variables.size());
				
				
			//	System.out.println("vehicleroutes" + vehicles.get(l.vehicle.number).vehicleRoutes);
			//	System.out.println(l.path);
			//	System.out.println("numRoutes"+numberOfRoutes);
				pathList.put(numberOfRoutes-1, l);
			//	System.out.println(pathList.get(numberOfRoutes-1).profit);
				
			//	col2.addTerm(visitedPickupsByVehicleOnRoute[l.vehicle.vehicleRoutes.lastElement()][l.vehicle.number][i], visitedPickupsCon[i]);
			//	col2.addTerm(1, oneVisitCon[l.vehicle.number]);

				model.update();
			

		}
		
		
		public void solveProblem() throws Exception {
			
			buildProblem();
			model.optimize();
			
			builder = new PathBuilder(pickupNodes, deliveryNodes, inputdata, pw, vehicles);
			
			//print
			for(int k = 0; k < vehicles.size(); k++) {
				for (GRBVar var : lambdaVars[k]) {
					System.out.println(var.get(GRB.StringAttr.VarName)  + " " +var.get(GRB.DoubleAttr.X));
				}
			}
			double[] dualVisitedPickupsCon = new double[pickupNodes.size()];  
			double[] dualOneVisitCon = new double[vehicles.size()];
			
			for(int i = 0; i < pickupNodes.size(); i++) {
			//	System.out.println("Pickup");
			//	System.out.println(pickupNodes.get(i).number);
			//	System.out.println(pickupNodes.get(i).location);
				double dualPickup_i = (double) visitedPickupsCon[i].get(GRB.DoubleAttr.Pi);
				dualVisitedPickupsCon[i] = dualPickup_i;
			//	System.out.println("DUAL: " + dualPickup_i);
				//System.out.println("HER");
				//System.out.println(dualVisitedPickupsCon.get(i));
				
			}
//			builder.dualVisitedPickupsCon = dualVisitedPickupsCon;
			
			for(int k = 0; k < vehicles.size(); k++) {
				double dualVehicle_k = (double) oneVisitCon[k].get(GRB.DoubleAttr.Pi);
				dualOneVisitCon[k]=dualVehicle_k;
			//	System.out.println("DUAL: " + dualVehicle_k);
				//System.out.println("HER");
				//System.out.println(dualOneVisitCon.get(k));
				
			}
//			builder.dualOneVisitCon = dualOneVisitCon;
			
			model.update();
			
			ArrayList<Label> bestLabels = new ArrayList<Label>();
//			double bestreducedCost = 100000000;
			boolean addedLabel = true;
			System.out.println("Objective value" +model.get(GRB.DoubleAttr.ObjVal));
			int counter = 0;
			while(addedLabel && counter<10000) {
				BBNode bbNode = new BBNode(null, 0, 0, vehicles, pickupNodes);
				counter++;
				addedLabel=false;
				System.out.println("Vehicle duals: "+Arrays.toString(dualOneVisitCon));
				System.out.println("Pickup duals: "+Arrays.toString(dualVisitedPickupsCon));
				System.out.println("");
				System.out.println("---New subproblem---");
				System.out.println("");
				for(int k = 0; k < vehicles.size(); k++) {
					
					System.out.println("");
					System.out.println("Solving subproblem for vehicle " + k);
					long startTime = System.nanoTime();
					Vector<Label> list = builder.BuildPaths(vehicles.get(k), dualVisitedPickupsCon, dualOneVisitCon, bbNode);
					bestLabels = builder.findBestLabel(list, bbNode);
					long endTime = System.nanoTime();
					System.out.println("Subproblem took "+(endTime - startTime)/1000000 + " milli seconds"); 
					
					if(bestLabels != null) {
					for(Label bestLabel : bestLabels) {
					//	if(bestLabel!=null) {
						bestLabel.routeNumber = numberOfRoutes;
						numberOfRoutes += 1;
							
						//	System.out.println("red cost " +bestLabel.reducedCost);
						//	System.out.println("Route number: " + numberOfRoutes);
						//	System.out.println("");
							
							
							
							vehicles.get(k).vehicleRoutes.add(numberOfRoutes);
						//	numberOfRoutes += 1;
						//	vehicles.get(l.vehicle.number).vehicleRoutes.add(numberOfRoutes);
						//	System.out.println ("HER: " +numberOfRoutes);
							addRoute(bestLabel);
							addedLabel=true;
					}
					 
					//addedLabel = false;
					} 
				// Kan flytte masterproblem-koden hit - løser da MP en gang per SP
				}	
				System.out.println("");	
				System.out.println("---Solving master problem---");
				System.out.println("");
				model.optimize();
				model.write("model.lp");		
				
				
				// Har løst masterproblem 
				// Sjekker om løsningen er heltallig 
				// Hvis ikke, kjør BB for å finne heltallig løsning 
				// Branch på de ikke-heltallige lambdaene - en branch til lambda = 0 og en til lambda = 1 
				// For lambda = 1: Løs subproblemet for den vehicle lambda gjelder med fikserte pickupnoder, dvs må ta de pickupene som blir tatt i lambda-ruten
				// For lambda = 0: Branch videre. Setter den andre lambdaen til vehicle lik 1 eller 0.
				// "Dominer" på profitt 		
					
					
					for(int i = 0; i < pickupNodes.size(); i++) {
						double dualPickup_i =  visitedPickupsCon[i].get(GRB.DoubleAttr.Pi);
						dualVisitedPickupsCon[i] = dualPickup_i;
					//	System.out.println("DUAL_pickup: " + dualPickup_i);
						//System.out.println("HER");
						//System.out.println(dualVisitedPickupsCon.get(i));
						
					}
	//				builder.dualVisitedPickupsCon = dualVisitedPickupsCon;
					for(int k = 0; k < vehicles.size(); k++) {
						double dualVehicle_k = oneVisitCon[k].get(GRB.DoubleAttr.Pi);
					//	System.out.println("dual of vehicle "+k+": "+dualVehicle_k);
						dualOneVisitCon[k]=dualVehicle_k;
		//				builder.dualOneVisitCon = dualOneVisitCon;
	//					System.out.println("DUAL_vehicle: " + dualVehicle_k);
	
					}
				model.update();
				
				MPsolutionVars = new Hashtable<Integer, Double>(); 
				for(int k = 0; k < vehicles.size(); k++) {
					int number = 0;
					int routeNumber = 0; 
					
					routesUtilized = new ArrayList<Integer>();
					
					
					
					//int roundCounter = 0;
					//int routeCounter = vehicles.get(k).vehicleRoutes.size()*roundCounter;
					//int routeCounter = 0;
				
					for (GRBVar var : lambdaVars[k]) {
						
						routeNumber = vehicles.get(k).vehicleRoutes.get(number);
						
						number ++;
					//	System.out.print("routenumber" + routeNumber);
					//	System.out.println("routeCouter" +routeNumber);
						if(var.get(GRB.DoubleAttr.X)>0.01 ) {
							System.out.println("");
							System.out.println(var.get(GRB.StringAttr.VarName)  + " " +var.get(GRB.DoubleAttr.X));
							MPsolutionVars.put(routeNumber, var.get(GRB.DoubleAttr.X));
							routesUtilized.add(routeNumber);
					//		int route = lambdaVars[k][r]
							//int route = 2;
							//if((routeCounter%2)!=0 && routeCounter > 2) {
							//	routeCounter++;
							//}
							System.out.println("Profit: " + pathList.get(routeNumber-1).profit);
							
						//	System.out.println("numRoutes"+numberOfRoutes);
							if(routeNumber>vehicles.size()) {
								System.out.println(pathList.get(routeNumber-1).toString());
								//routeNumber++;
								//System.out.println(pathList.get(routeNumber).profit);
							//	System.out.println("routeCouter2" +routeNumber);
								for(int i = 0; i < pathList.get(routeNumber-1).path.size(); i++) {
								//	System.out.println(pathList.get(routeNumber-1).path.get(i).number);
									
									//System.out.println(pathList.get(routeNumber-1).predesessor.toString());
									//System.out.println(pathList.get(routeNumber-1).toString());
								}
								Label temp = pathList.get(routeNumber-1).predesessor;
								while(temp!=null) {
									System.out.println(temp.toString());
								temp=temp.predesessor;
								}
							}
							
							
					
						}
					}
					//for(int i : routesUtilized) {
					//	System.out.println(i);
					//	System.out.println(MPsolutionVars.get(i));
					//}
					
				} 
				ArrayList<Node> fractionalPickupNodes = findFractionalNodes(MPsolutionVars);
				if(checkFractionality(fractionalPickupNodes) != null) {
					int indexCounter = 0;
					
					bbNode.setObjectiveValue(model.get(GRB.DoubleAttr.ObjVal));
					
					while(!fractionalPickupNodes.isEmpty()) {
						indexCounter ++;

						BBNode leftChild = new BBNode(bbNode, bbNode.getDepth()+1, indexCounter, vehicles, pickupNodes);
						removeIllegalLambdaVars(leftChild, this.lambdaVars);
						indexCounter ++;
						BBNode rightChild = new BBNode(bbNode, bbNode.getDepth()+1, indexCounter, vehicles, pickupNodes);
						Node branchingPickupNode = checkFractionality(fractionalPickupNodes);
						//branchingMatrixMaker
					//System.out.println("BrachingpickupNode" + branchingPickupNode.number + " " + branchingPickupNode.branchingVehicle.number );
						leftChild.branchingMatrix = branchingMatrixMaker(bbNode, branchingPickupNode, "left");
						
						rightChild.branchingMatrix = branchingMatrixMaker(bbNode, branchingPickupNode, "right");
						System.out.println(leftChild.branchingMatrix[3][2]);
						for(Vehicle v : vehicles) {
							//Label her = pathList.get(345);
							//System.out.println(her.pickupNodesVisited);
							Vector<Label> list = builder.BuildPaths(v, dualVisitedPickupsCon, dualOneVisitCon, leftChild);
							//System.out.println(list);
							
							bestLabels = builder.findBestLabel(list, bbNode);
						}
						//BBNode rightChild = branchingMatrixMaker();
					//	System.out.println("Fraction" + checkFractionality(fractionalPickupNodes).number);
					}
					
					
					
				}
				
				
		//		for(int i = 0; i < variables.size(); i++) {
		//			if(variables.get(i).getSol()>0.9) {
		//				System.out.println(variables.get(i).getSol()+" "+pathList.get(i).toString());
		//			}
				}
			
		//	for(int k = 0; k < vehicles.size(); k++) {
		//		for (int r : vehicles.get(k).vehicleRoutes) {
		//			if(var.get(GRB.DoubleAttr.X)>0.01) {
		//			System.out.println(pathList.get(r).path);
		//			}
		//		}
		//		}
			
			
			

		model.dispose();
	    env.dispose();
			
		}
		
		public ArrayList<Node> findFractionalNodes (Hashtable<Integer, Double> MPsolutionVars) throws Exception {
			ArrayList<Node> fractionalPickupNodes = new ArrayList<Node>();
			System.out.println(MPsolutionVars.toString());
			for (Node node : pickupNodes) {
				node.fraction = 0;
				for(Vehicle vehicle : vehicles) {
					//System.out.println(vehicle.vehicleRoutes);
					double vehicleFraction = 0;
					for(int route : vehicle.vehicleRoutes) {
						
						if (MPsolutionVars.containsKey(route) && MPsolutionVars.get(route) < 1  ) {
						
							if (pathList.get(route).path.contains(node)) {
								//System.out.println(route);
								//System.out.println(node.number);
								//System.out.println(vehicleFraction);
								vehicleFraction += MPsolutionVars.get(route);
								
								//System.out.println(vehicleFraction);
							//	System.out.println(vehicleFraction);
							}
						}
					}	
					if (vehicleFraction > node.fraction) {
						node.fraction = vehicleFraction;
						node.branchingVehicle = vehicle;
					}
				}
				if (node.fraction < 1 - zeroTol && node.fraction > 0 + zeroTol ) {
					fractionalPickupNodes.add(node);
					
				}System.out.println(node.number + " " + node.fraction);
				
			}
			return fractionalPickupNodes;
		}
		
		public Node checkFractionality (ArrayList<Node> fractionalPickupNodes) throws Exception {
		
			//System.out.println(fractionalPickupNodes);
			Node mostFractionalNode = null;
			double biggestFraction = 0;
			double fraction;
			for (Node n : fractionalPickupNodes) {
				if (n.fraction <= 0.5) {
					fraction =  n.fraction;
				}
				else {
					fraction = 1 - n.fraction;
				}
				if (fraction > biggestFraction) {
					biggestFraction = fraction;
					mostFractionalNode = n;	
				}				
			}
			if(!fractionalPickupNodes.isEmpty()) {
					fractionalPickupNodes.remove(mostFractionalNode);
					System.out.println("MOST FRACTIONAL " + mostFractionalNode.number);
					System.out.println("vehicle " + mostFractionalNode.branchingVehicle.number);
			}
			
			return mostFractionalNode;
		}
		
		
		public int[][] branchingMatrixMaker (BBNode bbNode, Node branchingPickupNode, String child) throws Exception {
			int[][] branchingMatrix = bbNode.branchingMatrix;
			if(child == "left") {
				branchingMatrix[branchingPickupNode.branchingVehicle.number][(branchingPickupNode.number/2) - 1] = -1;
				return branchingMatrix;
			}
			else if(child == "right") {
				for(Vehicle v : vehicles) {
					if(branchingPickupNode.branchingVehicle.number == v.number) {
						branchingMatrix[branchingPickupNode.branchingVehicle.number][(branchingPickupNode.number/2) - 1] = 1;
					}
					else {
						branchingMatrix[v.number][(branchingPickupNode.number/2) - 1] = -1;
					}	
				}
			}
			return branchingMatrix;
		}
		
		
		public void removeIllegalLambdaVars(BBNode bbNode, ArrayList<GRBVar>lambdaVars[]) throws Exception{
			ArrayList<GRBVar> legalLambdaVars = new ArrayList<GRBVar>();
			for (Vehicle v : vehicles) {
				int number = 0;
				int routeNumber; 
					for (GRBVar var : lambdaVars[v.number]) {			
						routeNumber = vehicles.get(v.number).vehicleRoutes.get(number);
						number ++;
						int pickupNumber = 2;
						for (int pickup : bbNode.branchingMatrix[v.number]) {
							
							System.out.println(pathList.get(4).profit);
							if (pathList.get(routeNumber-1).pickupNodesVisited != null && pathList.get(routeNumber-1).pickupNodesVisited.contains(pickupNumber)) {
								if (pickup == 1) {
									var.set(GRB.DoubleAttr.LB, 1);
									var.set(GRB.DoubleAttr.UB, 1);
								}
								else if(pickup == -1) {
									var.set(GRB.DoubleAttr.LB, 0);
									var.set(GRB.DoubleAttr.UB, 0);
								}
								//print
								if (MPsolutionVars.get(var.get(GRB.DoubleAttr.X)) != null) {
								System.out.println(routeNumber);
								System.out.println(MPsolutionVars.get(var));
								}
							}
							pickupNumber += 2;
						
					}
				}
			}
		}
		
	}

    // Optimize model

 //  

  //  System.out.println(lambda.get(GRB.StringAttr.VarName)
//                         + " " +lambda.get(GRB.DoubleAttr.X));
   // System.out.println(y.get(GRB.StringAttr.VarName)
    //                   + " " +y.get(GRB.DoubleAttr.X));
   // System.out.println(z.get(GRB.StringAttr.VarName)
    //                   + " " +z.get(GRB.DoubleAttr.X));

   // System.out.println("Obj: " + model.get(GRB.DoubleAttr.ObjVal));
		  
	//   }