import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;
import java.util.Vector;
import gurobi.*;

	public class GurobiInterface {
		
		private Hashtable<Integer, Label> pathList;
	//	public Hashtable<Integer, Double> MPsolutionVars;
	//	public ArrayList<Integer> routesUtilized;
		double[] dualVisitedPickupsCon;  
		double[] dualOneVisitCon;
		double totalTimeInSub = 0;
		double totalTimeInMaster = 0;
		long maxExecutionTime = 10800000;
		long rootNodeTime;
		double optimalObjective = 0;
		double rootNodeObjective = 0;
		double bestLeafNodeObjective = 0;
		double bestIntegerLeafNodeObjective = 0;
		int numberBBnodes = 0;
		int numberOfSubproblemCalls;
		double numberOfPickupsServedInBestNode;
		double numberOfPickupsServedInRootNode;
		double numberOfPickupsServed = 0;

		
		// Creating Gurobi environment
	    GRBEnv    env   = new GRBEnv();
	    GRBModel  model;
	    
	    // Creating Gurobi variables
	    private Vector<GRBVar> variables;
		public ArrayList<GRBVar>[] lambdaVars;
		ArrayList<BBNode> leafNodes; 
	    
	    // Creating Gurobi constraints
		public GRBConstr[] visitedPickupsCon;
		public GRBConstr[] oneVisitCon; 
		
		public GRBLinExpr visitedPickupsLeftHand[];
		public GRBLinExpr oneVisitLeftHand[];
		
		// Creating Gurobi objective function
		public GRBLinExpr objective;
		
		public double zeroTol = 0.0001;
	    public InstanceData inputdata;
		public double profit = 0;
		public Vector<Node> pickupNodes;
		public Vector<Node> deliveryNodes;
		public Vector<Node> nodesWithoutDepot;
		public PrintWriter pw;
		public PrintWriter fw;
		public Vector<Node> path;
		public Vector<Vehicle> vehicles;
		public PathBuilder builder;
		public int numberOfRoutes = 0;
		int BBNodeIDcounter = 1;
		long solutionStartTime;

		
		public GurobiInterface(InstanceData inputdata, Vector<Node> deliveryNodes, Vector<Node> pickupNodes, Vector<Node> nodesWithoutDepot, Vector<Vehicle> vehicles, Vector<Double> dualVisitedPickupsCon, Vector<Double> dualOneVisitCon, PrintWriter pw, PrintWriter fw) throws Exception {
			env.set(GRB.IntParam.Presolve, 0);
			env.set(GRB.DoubleParam.OptimalityTol, 0.000000001);
			env.set(GRB.DoubleParam.FeasibilityTol, 0.000000001);
			model = new GRBModel(env);
			
			this.vehicles = vehicles; 
			this.inputdata = inputdata;
			this.pickupNodes = pickupNodes;
			this.deliveryNodes = deliveryNodes;
			this.nodesWithoutDepot = nodesWithoutDepot;
			this.pw = pw;
			this.fw = fw;
			this.vehicles = vehicles;
			this.pathList = new Hashtable<Integer, Label>();
			this.leafNodes = new ArrayList<>();	
		}
		
		public void buildProblem() throws Exception {
			this.objective = new GRBLinExpr();
			this.variables = new Vector<GRBVar>();
			this.lambdaVars = new ArrayList[vehicles.size()];
			this.visitedPickupsCon = new GRBConstr[pickupNodes.size()];
			this.oneVisitCon = new GRBConstr[vehicles.size()];
			this.oneVisitLeftHand = new GRBLinExpr[vehicles.size()];
			this.visitedPickupsLeftHand = new GRBLinExpr[pickupNodes.size()];

			Label firstLabel = new Label();
			firstLabel.profit = 0;
		
			for(int k = 0; k < vehicles.size(); k++) {
				this.lambdaVars[k] = new ArrayList<GRBVar>();
				this.vehicles.get(k).vehicleRoutes = new Vector<Integer>();
				vehicles.get(k).vehicleRoutes.add(numberOfRoutes);
				for (int r : vehicles.get(k).vehicleRoutes) {
					GRBVar temp = model.addVar(0, GRB.INFINITY, firstLabel.profit, GRB.CONTINUOUS, "lambda_"+k+"_"+r); //fjernet col her
					this.lambdaVars[k].add(temp);
					pathList.put(numberOfRoutes, firstLabel);
					this.objective.addTerm(firstLabel.profit, temp);
					GRBLinExpr temp2 = new GRBLinExpr();
					temp2.addTerm(1, temp);
					this.oneVisitCon[k] = model.addConstr(temp2, GRB.EQUAL, 1, "oneVisitCon"+k);		// skal egentlig være Equal 
				}
				numberOfRoutes += 1;
			}
			model.setObjective(objective, GRB.MAXIMIZE);
			

			
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

		}
		
		
		
		public void addRoute(Label l) throws Exception{
			GRBVar tempVar = model.addVar(0, GRB.INFINITY, l.profit, GRB.CONTINUOUS,  "lambda_"+l.vehicle.number + "_" + numberOfRoutes);
			lambdaVars[l.vehicle.number].add(tempVar);
			this.objective.addTerm(l.profit, tempVar);
			for(int i = 0; i < pickupNodes.size(); i++) {	
				if(l.pickupNodesVisited.contains(pickupNodes.get(i).number)) {
					model.chgCoeff(visitedPickupsCon[i], tempVar, 1);
				}	
			}
				model.chgCoeff(oneVisitCon[l.vehicle.number], tempVar, 1);
				pathList.put(numberOfRoutes, l);
		}
		
		
		public void columnGenerator() throws Exception {
			solutionStartTime = System.currentTimeMillis();
			
			initiateProblem();
			int[][] initialBranchingMatrix = new int[vehicles.size()][pickupNodes.size()];
			BBNode rootNode = new BBNode(null, 0, 0, vehicles, pickupNodes, initialBranchingMatrix, "root");
			long startRootNodeTime = System.currentTimeMillis();
			solveProblem(rootNode);
			long endRootNodeTime = System.currentTimeMillis();
			rootNodeTime =  (endRootNodeTime - startRootNodeTime);
			pw.println("Total time to solve rootnode: " + rootNodeTime );
			pw.println("  ");
			pw.println("---- ROOTNODE FINISHED ----");
			pw.println("  ");
			System.out.println("  ");
			System.out.println("---- ROOTNODE FINISHED ----");
			System.out.println("  ");
			rootNodeObjective = rootNode.getObjectiveValue();
			optimalObjective = rootNode.getObjectiveValue();
			numberOfPickupsServedInRootNode = findNumberOfPickupsServed(rootNode);
			pw.println("number" + numberOfPickupsServedInRootNode);
			ArrayList<Node> fractionalPickupNodes = findFractionalNodes(rootNode.MPsolutionVarsBBnode);
	//		numberOfPickupsServedInRootNode = fractionalPickupNodes.size();
			if(checkPickupFractionality(fractionalPickupNodes) != null) {
				pw.println("  ");
				pw.println("---- BRANCHING ----");
				pw.println("  ");
				System.out.println("  ");
				System.out.println("---- BRANCHING ----");
				System.out.println("  ");
				buildTree(rootNode);
				pw.println("Total time in subproblem: " + totalTimeInSub);
				System.out.println("Total time in subproblem: " + totalTimeInSub);
				pw.println("Total time in master problem " + totalTimeInMaster);
				System.out.println("Total time in master problem " + totalTimeInMaster);
				//solveBBnode(rootNode);
				
				//while() {
					
				//}
				// Kall BB-tre
			}
			model.dispose();
		    env.dispose();
		}
		
		public void initiateProblem() throws Exception {
	//		long startTime = System.nanoTime();
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
			System.out.println("Objective value" +model.get(GRB.DoubleAttr.ObjVal));
			pw.println("Objective value" +model.get(GRB.DoubleAttr.ObjVal));
			model.update();
	//		long endTime = System.nanoTime();
	//		totalTimeInMaster += (endTime-startTime)/1000000;
		}
		
		public void solveProblem(BBNode bbNode) throws Exception {
		
		
			removeIllegalLambdaVars(bbNode, this.lambdaVars);
			ArrayList<Label> bestLabels = new ArrayList<Label>();
		//	Label bestLabel;
//			double bestreducedCost = 100000000;
			boolean addedLabel = true;
			
			int counter = 0;
		//	BBNode rootNode = null;
			dualVisitedPickupsCon = new double[pickupNodes.size()];  
			dualOneVisitCon = new double[vehicles.size()];
			while(addedLabel && counter<10000) {
				long startTimeMaster = 0;
				pw.println("NEW MP");
				System.out.println("NEW MP");
		//		rootNode = new BBNode(null, 0, 0, vehicles, pickupNodes);
				counter++;
				addedLabel=false;
				System.out.println("Vehicle duals: "+Arrays.toString(dualOneVisitCon));
				System.out.println("Pickup duals: "+Arrays.toString(dualVisitedPickupsCon));
				System.out.println("");
				System.out.println("---New subproblem---");
				System.out.println("");
				for(int k = 0; k < vehicles.size(); k++) {
					if(System.currentTimeMillis() - solutionStartTime > maxExecutionTime) {
						printResults();
						fw.close();
						pw.close();
					//	System.out.println(System.currentTimeMillis());
					//	System.out.println(solutionStartTime);
					//	System.out.println(System.currentTimeMillis()-solutionStartTime);
						System.out.println("STOPPING");
						pw.print(maxExecutionTime);
						System.exit(0);
						
					}
					System.out.println("");
					System.out.println("Solving subproblem for vehicle " + k);
					long startTime = System.nanoTime();
					Vector<Label> list = builder.BuildPaths(vehicles.get(k), dualVisitedPickupsCon, dualOneVisitCon, bbNode);
					bestLabels = builder.findBestLabel(list, bbNode, pathList, vehicles.get(k));
					long endTime = System.nanoTime();
					numberOfSubproblemCalls += 1;
					
				//	totalTimeInSub = (endTime - startTime)/1000000;
					System.out.println("Subproblem took "+(endTime-startTime)/1000000 + " milli seconds"); 
					totalTimeInSub += (endTime-startTime)/1000000;
		//			startTimeMaster = System.nanoTime();
					if(bestLabels != null) {
					//if(bestLabel != null) {
						for(Label bestLabel : bestLabels) {
						//	if(bestLabel!=null) {
							bestLabel.routeNumber = numberOfRoutes;
							
								
							//	System.out.println("red cost " +bestLabel.reducedCost);
							//	System.out.println("Route number: " + numberOfRoutes);
							//	System.out.println("");
								
							//	System.out.println("dette tar lang tid");
								
								vehicles.get(k).vehicleRoutes.add(numberOfRoutes);
								
							//	numberOfRoutes += 1;
							//	vehicles.get(l.vehicle.number).vehicleRoutes.add(numberOfRoutes);
							//	System.out.println ("HER: " +numberOfRoutes);
								addRoute(bestLabel);
								addedLabel=true;
								numberOfRoutes += 1;
						}
		//			long endTimeMaster = System.nanoTime(); 
		//			totalTimeInMaster += (endTimeMaster-startTimeMaster)/1000000;
					//addedLabel = false;
					} 
				// Kan flytte masterproblem-koden hit - løser da MP en gang per SP
				}	
				System.out.println("");	
				System.out.println("---Solving master problem---");
				System.out.println("");
				model.update();
				long startTimeSolveMaster = System.nanoTime(); 
				model.optimize();
				long endTimeSolveMaster = System.nanoTime();
				totalTimeInMaster += (endTimeSolveMaster-startTimeSolveMaster)/1000000;
				
				bbNode.setObjectiveValue(model.get(GRB.DoubleAttr.ObjVal));
		//		model.write("model.lp");		
				pw.println("Objective value" + model.get(GRB.DoubleAttr.ObjVal));
				
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
			
			}
		
			//	MPsolutionVars = new Hashtable<Integer, Double>(); 
				for(int k = 0; k < vehicles.size(); k++) {
					int number = 0;
					int routeNumber = 0; 
					
				//	routesUtilized = new ArrayList<Integer>();
					
					for (GRBVar var : lambdaVars[k]) {
						
						routeNumber = vehicles.get(k).vehicleRoutes.get(number);
						
						number ++;
					//	System.out.print("routenumber" + routeNumber);
					//	System.out.println("routeCouter" +routeNumber);
						if(var.get(GRB.DoubleAttr.X)>0.01 ) {
							//System.out.println(pathList.get(20).profit);
							//System.out.println(routeNumber);
							System.out.println("");
							pw.println("");
							System.out.println(var.get(GRB.StringAttr.VarName)  + " " +var.get(GRB.DoubleAttr.X));
							pw.println(var.get(GRB.StringAttr.VarName)  + " " +var.get(GRB.DoubleAttr.X));
						//	MPsolutionVars.put(routeNumber, var.get(GRB.DoubleAttr.X));
						//	routesUtilized.add(routeNumber);
							bbNode.lambdaValues.add(var.get(GRB.DoubleAttr.X));
							bbNode.MPsolutionVarsBBnode.put(routeNumber, var.get(GRB.DoubleAttr.X));
					//		int route = lambdaVars[k][r]
							//int route = 2;
							//if((routeCounter%2)!=0 && routeCounter > 2) {
							//	routeCounter++;
							//}
						//	System.out.println("Profit: " + pathList.get(routeNumber).profit);
						//	pw.println("Profit: " + pathList.get(routeNumber).profit);
							
						//	System.out.println("numRoutes"+numberOfRoutes);
						//	if(routeNumber>vehicles.size()-1) {
						//		System.out.println(pathList.get(routeNumber).toString());
						//		pw.println(pathList.get(routeNumber).toString());
								//routeNumber++;
								//System.out.println(pathList.get(routeNumber).profit);
							//	System.out.println("routeCouter2" +routeNumber);
						//		for(int i = 0; i < pathList.get(routeNumber).path.size(); i++) {
								//	System.out.println(pathList.get(routeNumber-1).path.get(i).number);
									
									//System.out.println(pathList.get(routeNumber-1).predesessor.toString());
									//System.out.println(pathList.get(routeNumber-1).toString());
						//		}
						//		Label temp = pathList.get(routeNumber).predesessor;
						//		while(temp!=null) {
						//			System.out.println(temp.toString());
						//			pw.println(temp.toString());
									
						//		temp=temp.predesessor;
							
						//		}
							
							printSolution(routeNumber);
					
						
						}
							
							
					
						}
					}
					//for(int i : routesUtilized) {
					//	System.out.println(i);
					//	System.out.println(MPsolutionVars.get(i));
					//}
				 
				
			
		
	
				
		//		for(int i = 0; i < variables.size(); i++) {
		//			if(variables.get(i).getSol()>0.9) {
		//				System.out.println(variables.get(i).getSol()+" "+pathList.get(i).toString());
		//			}
				
			
			
		//	for(int k = 0; k < vehicles.size(); k++) {
		//		for (int r : vehicles.get(k).vehicleRoutes) {
		//			if(var.get(GRB.DoubleAttr.X)>0.01) {
		//			System.out.println(pathList.get(r).path);
		//			}
		//		}
		//		}
		
			resetIllegalLambdaVars(this.lambdaVars);
		}
			
		 
		
		
		public void printSolution(int route) throws Exception{
		//	for(int k = 0; k < vehicles.size(); k++) {
		//		int number = 0;
		//		int routeNumber = 0; 
				
			//	routesUtilized = new ArrayList<Integer>();
				
			//	for (GRBVar var : lambdaVars[k]) {
			//		
			//		routeNumber = vehicles.get(k).vehicleRoutes.get(number);
					
			//		number ++;
			for(Vehicle v : vehicles ) {
			//	for(int route : bbNode.MPsolutionVarsBBnode.keySet()) {
						
							if(v.vehicleRoutes.contains(route)) {
								System.out.println("Vehicle " + v.number + " drives route " + route);
								pw.println("Vehicle " + v.number + " drives route " + route);
								System.out.println("Profit: " + pathList.get(route).profit);
								pw.println("Profit: " + pathList.get(route).profit);
								
								if(route>vehicles.size()-1) {
									System.out.println(pathList.get(route).toString());
									pw.println(pathList.get(route).toString());
								Label temp = pathList.get(route).predesessor;
								while(temp!=null) {
									System.out.println(temp.toString());
									pw.println(temp.toString());
									
								temp=temp.predesessor;
								}
								}
								System.out.println(" ");
								pw.println(" ");
							}
						
						
					//}
							}
				//	System.out.print("routenumber" + routeNumber);
				//	System.out.println("routeCouter" +routeNumber);
				//	if(var.get(GRB.DoubleAttr.X)>0.01 ) {
						//System.out.println(pathList.get(20).profit);
						//System.out.println(routeNumber);
				//		System.out.println("");
				//		pw.println("");
				//		System.out.println(var.get(GRB.StringAttr.VarName)  + " " +var.get(GRB.DoubleAttr.X));
				//		pw.println(var.get(GRB.StringAttr.VarName)  + " " +var.get(GRB.DoubleAttr.X));
					//	MPsolutionVars.put(routeNumber, var.get(GRB.DoubleAttr.X));
					//	routesUtilized.add(routeNumber);
					//	bbNode.lambdaValues.add(var.get(GRB.DoubleAttr.X));
					//	bbNode.MPsolutionVarsBBnode.put(routeNumber, var.get(GRB.DoubleAttr.X));
				//		int route = lambdaVars[k][r]
						//int route = 2;
						//if((routeCounter%2)!=0 && routeCounter > 2) {
						//	routeCounter++;
						//}
						
						
					//	System.out.println("numRoutes"+numberOfRoutes);
					//	if(routeNumber>vehicles.size()-1) {
					//		System.out.println(pathList.get(routeNumber).toString());
					//		pw.println(pathList.get(routeNumber).toString());
							//routeNumber++;
							//System.out.println(pathList.get(routeNumber).profit);
						//	System.out.println("routeCouter2" +routeNumber);
						//	for(int i = 0; i < pathList.get(routeNumber).path.size(); i++) {
							//	System.out.println(pathList.get(routeNumber-1).path.get(i).number);
								
								//System.out.println(pathList.get(routeNumber-1).predesessor.toString());
								//System.out.println(pathList.get(routeNumber-1).toString());
					//		}
							
					//	}
						
						
				
					//}
			//	}
				//for(int i : routesUtilized) {
				//	System.out.println(i);
				//	System.out.println(MPsolutionVars.get(i));
				//}
			//} 
		}
		
		public ArrayList<Node> findFractionalNodes (Hashtable<Integer, Double> MPsolutionVars) throws Exception {
			ArrayList<Node> fractionalPickupNodes = new ArrayList<Node>();
		//	System.out.println(MPsolutionVars.toString());
			for (Node node : pickupNodes) {
				node.fraction = 0;
			//	node.branchingVehicle.number = -1;
				for(Vehicle vehicle : vehicles) {
					//System.out.println(vehicle.vehicleRoutes);
					double vehicleFraction = 0;
					for(int route : vehicle.vehicleRoutes) {
						
						if (MPsolutionVars.containsKey(route) && MPsolutionVars.get(route) < 1  ) {
							//System.out.println(route);
							if(route >= vehicles.size()){
								if (pathList.get(route).path.contains(node)) {
									//System.out.println(route);
									//System.out.println(node.number);
									//System.out.println(vehicleFraction);
						//			System.out.println(node.number);
									vehicleFraction += MPsolutionVars.get(route);
							//		System.out.println(MPsolutionVars.get(route));
							//		System.out.println(route);
									
									
									//System.out.println(vehicleFraction);
								//	System.out.println(vehicleFraction);
								}
							}
						}
					}	
					if (vehicleFraction > node.fraction + zeroTol) {
						node.fraction = vehicleFraction;
						node.branchingVehicle = vehicle;
					}
				}
				if (node.fraction < 1 - zeroTol && node.fraction > 0 + zeroTol ) {
					fractionalPickupNodes.add(node);
					pw.println(node.number + " " + node.fraction + " " + node.branchingVehicle.number);
					
				}//System.out.println(node.number + " " + node.fraction + " " + node.branchingVehicle.number );
				
			}
			return fractionalPickupNodes;
		}
		
		public double findNumberOfPickupsServed (BBNode bbNode) throws Exception{
			double currentNumber = 0;
			numberOfPickupsServed = 0;
			for(int key : bbNode.MPsolutionVarsBBnode.keySet()) {
				currentNumber = ( bbNode.MPsolutionVarsBBnode.get(key) * pathList.get(key).pickupNodesVisited.size()) ;
				numberOfPickupsServed += currentNumber;
				pw.println(key);
				pw.println(numberOfPickupsServed);
				pw.println(bbNode.MPsolutionVarsBBnode.get(key));
				pw.println(pathList.get(key).pickupNodesVisited.size());
			}
			
			return numberOfPickupsServed;
		} 
		
		public Node checkPickupFractionality (ArrayList<Node> fractionalPickupNodes) throws Exception {
		
			//System.out.println(fractionalPickupNodes);
			Node mostFractionalNode = null;
			double biggestFraction = 1;
			double fraction;
			for (Node n : fractionalPickupNodes) {
				if (n.fraction >= 0.5) {
					fraction =  1 - n.fraction;
				}
				else {
					fraction = 0.5 -  n.fraction ;
				}
				if (fraction < biggestFraction) {
					biggestFraction = fraction;
					mostFractionalNode = n;	
					System.out.println("Node: " + n.number + " frac: " + fraction + " vehicle: " + n.branchingVehicle.number);
					
					
				}				
			}
			if(!fractionalPickupNodes.isEmpty()) {
					
					System.out.println("MOST FRACTIONAL " + mostFractionalNode.number);
					pw.println("MOST FRACTIONAL " + mostFractionalNode.number);
					System.out.println("vehicle " + mostFractionalNode.branchingVehicle.number);
					pw.println("vehicle " + mostFractionalNode.branchingVehicle.number);
			}
			
			return mostFractionalNode;
		}
		
		
		public int[][] branchingMatrixMaker (BBNode bbNode, Node branchingPickupNode, String type) throws Exception {

			int[][] branchingMatrix2 = new int[vehicles.size()][pickupNodes.size()];
		//	int[][] branchingMatrix2 = bbNode.branchingMatrix;
			
			for (int i = 0; i < bbNode.branchingMatrix.length; i++) {
			    for (int j = 0; j < bbNode.branchingMatrix[i].length; j++) {
			    	branchingMatrix2[i][j] = bbNode.branchingMatrix[i][j];
			    }
			}
			
			if(type == "left") {
				branchingMatrix2[branchingPickupNode.branchingVehicle.number][(branchingPickupNode.number/2) - 1] = -1;
				//return branchingMatrix;
			}
			else if(type == "right") {
				for(Vehicle v : vehicles) {
					if(branchingPickupNode.branchingVehicle.number == v.number) {
						branchingMatrix2[branchingPickupNode.branchingVehicle.number][(branchingPickupNode.number/2) - 1] = 1;
					}
					else {
						branchingMatrix2[v.number][(branchingPickupNode.number/2) - 1] = -1;
					}	
				}
			//	return branchingMatrix;
			}
			return branchingMatrix2;
		}
		
		// Forcing lambdas that must be removed due to branching restrictions, to zero
		public void removeIllegalLambdaVars(BBNode bbNode, ArrayList<GRBVar>lambdaVars[]) throws Exception{
			for (Vehicle v : vehicles) {
				int number = 0;
				int routeNumber; 
				for (GRBVar var : lambdaVars[v.number]) {			
					routeNumber = vehicles.get(v.number).vehicleRoutes.get(number);
					
					// If there is a right child (forcing some pickup to 1), the initialization routes (going from depot to depot) must be set to zero 
					if(bbNode.type.equals("right") && number == 0) {
						pw.println(var.get(GRB.StringAttr.VarName)) ;
						var.set(GRB.DoubleAttr.LB, 0);
						var.set(GRB.DoubleAttr.UB, 0);
						model.update();
					}
					number ++;
					
					// Going through all pickup nodes 
					for(Node pickupNumber : pickupNodes) {
						// If the pickup node is contained in the route of a lambda and the branching matrix of the current node says that this pickup node cannot be visited (= -1), force the lambda to zero 
						if (pathList.get(routeNumber).pickupNodesVisited != null && pathList.get(routeNumber).pickupNodesVisited.contains(pickupNumber.number) && bbNode.branchingMatrix[v.number][pickupNumber.number/2 - 1]  == -1) {
							var.set(GRB.DoubleAttr.LB, 0);
							var.set(GRB.DoubleAttr.UB, 0);
							model.update();
							break;
						}
						// If the pickup node is not contained in the route a lambda and the branching matrix of the current node says that this pickup node cannot be visited (= 1), force the lambda to zero 
						else if (pathList.get(routeNumber).pickupNodesVisited != null && !pathList.get(routeNumber).pickupNodesVisited.contains(pickupNumber.number) && bbNode.branchingMatrix[v.number][pickupNumber.number/2 - 1] == 1) {
								var.set(GRB.DoubleAttr.LB, 0);
								var.set(GRB.DoubleAttr.UB, 0);
								model.update();
								break;
						}
					}	
				}
			} 
		}
		
		
		// If the root node is fractional, this method is called in order to build the branch and bound tree to find the optimal integer solution
		public void buildTree (BBNode rootNode) throws Exception {
			// Adds the leaf node to the set of leaf nodes
			leafNodes.add(rootNode);
			boolean fractional = true;	
			BBNode bestBBNode = null;
			
			// Stay in the while as long as the bestBBNode (BBnode with best objective value) is fractional 
			while(fractional) {
				double bestProfit = 0;
				
				// Go through all leaf nodes and check which has best objective value 
				for(BBNode leafNode : leafNodes) {
					if(leafNode.getObjectiveValue() > bestProfit) {
						bestProfit = leafNode.getObjectiveValue();
						bestBBNode = leafNode;
					}
				}
				
				// Print the information about the current best BBnode to file
				pw.println("BESTNODE: " + bestBBNode.getNodeId() + " " + bestBBNode.getObjectiveValue());
				pw.println("Branching matrix bestBBnode: ");
				for (int i = 0; i < bestBBNode.branchingMatrix.length; i++) {
				    for (int j = 0; j < bestBBNode.branchingMatrix[i].length; j++) {
				        pw.print(bestBBNode.branchingMatrix[i][j] + " ");
				    }
				    pw.println();
				}
				
				// Check if the current best BBNode is integer, and if so, terminate and retur the current best BBNode as the solution
				if(!checkLambdaFractionality(bestBBNode)) {	
					System.out.println("BESTNODE: " + bestBBNode.getNodeId() + " " + bestBBNode.getObjectiveValue());
					pw.println("BESTNODE: " + bestBBNode.getNodeId() + " " + bestBBNode.getObjectiveValue());
					System.out.println(" ");
					pw.println(" ");
					optimalObjective = bestBBNode.getObjectiveValue();
				//	numberOfPickupsServedInBestNode = bestBBNode.numberOfPickupsServed;
					for(int route : bestBBNode.MPsolutionVarsBBnode.keySet()) {
						printSolution(route);
					}
					numberOfPickupsServedInBestNode = findNumberOfPickupsServed(bestBBNode);
					fractional = false;
				}
				
				// If the current best BBNode is fractional, branch and create two new child nodes
				else {
					// Finds which pickup nodes in the solution of the current BBnode that are fractional, and identifies the branchingPickupNode as the node that is most fractional
					ArrayList<Node> fractionalPickupNodes = findFractionalNodes(bestBBNode.MPsolutionVarsBBnode);
					Node branchingPickupNode = checkPickupFractionality(fractionalPickupNodes);
					
					// Remove bestBBnode from leaf nodes before branching
					leafNodes.remove(bestBBNode);
					
					// Add the branching node to the list of pickup nodes branched of the current BBNode
					bestBBNode.pickupNodesBranchedOn.add(branchingPickupNode.number);
					
					// Creating the left child based on the branching pickup node 
					BBNode leftChild = new BBNode(bestBBNode, bestBBNode.getDepth()+1, BBNodeIDcounter, vehicles, pickupNodes, branchingMatrixMaker(bestBBNode, branchingPickupNode, "left"), "left");
					
					BBNodeIDcounter += 1;
					
					// Adding the pickup nodes branched on from the parent node to the pickup nodes branched on of the child node
					for (int pickup : bestBBNode.pickupNodesBranchedOn) {
						leftChild.pickupNodesBranchedOn.add(pickup);	
					}
					
					// Print information about left child to file
					pw.println("-------------- LEFT CHILD RESULT -------------------");
					pw.println("ID left child " + leftChild.getNodeId());
					pw.println("Branching matrix left child: ");
					for (int i = 0; i < leftChild.branchingMatrix.length; i++) {
					    for (int j = 0; j < leftChild.branchingMatrix[i].length; j++) {
					        pw.print(leftChild.branchingMatrix[i][j] + " ");
					    }
					    pw.println();
					}
					
					// Solve the problem for the left child
					solveProblem(leftChild);
				
					// Creating the right child based on the branching pickup node 
					BBNode rightChild = new BBNode(bestBBNode, bestBBNode.getDepth()+1, BBNodeIDcounter, vehicles, pickupNodes, branchingMatrixMaker(bestBBNode, branchingPickupNode, "right"), "right");
					
					BBNodeIDcounter += 1;
					
					// Adding the pickup nodes branched on from the parent node to the pickup nodes branched on of the child node
					for (int pickup : bestBBNode.pickupNodesBranchedOn) {
						rightChild.pickupNodesBranchedOn.add(pickup);	
					}
					
					// Print information about right child to file
					pw.println("---------------- RIGHT CHILD RESULT -------------------");
					pw.println("ID right child " + rightChild.getNodeId());
					pw.println("branchingmatrix right child: ");
					for (int i = 0; i < rightChild.branchingMatrix.length; i++) {
					    for (int j = 0; j < rightChild.branchingMatrix[i].length; j++) {
					        pw.print(rightChild.branchingMatrix[i][j] + " ");
					    }
					    pw.println();
					}
				
					// Solve the problem for the right child
					solveProblem(rightChild);
					
					// Add left and right child to leaf nodes
					leafNodes.add(leftChild);
					leafNodes.add(rightChild);
				}
			}
		}
			

		// Method resetting lambda variables that are forced to 0, back to between 0 and 1 (used in the end of the solveProblem method)
		public void resetIllegalLambdaVars(ArrayList<GRBVar>lambdaVars[]) throws Exception {
			for (Vehicle v : vehicles) {
				for (GRBVar var : lambdaVars[v.number]) {
					var.set(GRB.DoubleAttr.LB, 0);
					var.set(GRB.DoubleAttr.UB, 1);
					model.update();
				}
			}
		}
		
		
		// Method checking whether a one or more of the lambda variables of a branch and bound node is fractional
		public boolean checkLambdaFractionality (BBNode bbNode) {
			for (double lambda : bbNode.lambdaValues) {
				if (lambda != 0 && lambda != 1) {
					return true;
				}	
			}
			return false;	
		}
		
		
		// Method printing key numbers from each column generation run to a common file for all runs 
		public void printResults () {
			for(BBNode leafNode : leafNodes) {
				// Finding the objective of the best leaf node that is not integer
				if(checkLambdaFractionality(leafNode) && leafNode.getObjectiveValue() > bestLeafNodeObjective ) {
					bestLeafNodeObjective = leafNode.getObjectiveValue();
				}
				// Finding the objective of the best leaf node that is integer
				else if(!checkLambdaFractionality(leafNode) && leafNode.getObjectiveValue() > bestIntegerLeafNodeObjective) {
					bestIntegerLeafNodeObjective = leafNode.getObjectiveValue();
				}
			}
			// Print to results file
			fw.print(optimalObjective + ";" + rootNodeObjective + ";" + bestLeafNodeObjective + ";" + bestIntegerLeafNodeObjective + ";" + BBNodeIDcounter + ";" + numberOfRoutes + ";" + numberOfSubproblemCalls + ";" + builder.numberOfDominatedLabels + ";" + builder.numberOfPaths + ";" + numberOfPickupsServedInRootNode + ";" + numberOfPickupsServedInBestNode + ";" + totalTimeInMaster + ";" + totalTimeInSub + ";" + rootNodeTime + ";");
		}
			
}

