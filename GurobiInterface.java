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
		
		// Creating Gurobi environment
	    GRBEnv    env   = new GRBEnv("mip1.log");
	    GRBModel  model;
	    
	    // Creating Gurobi variables
	    private Vector<GRBVar> variables;
//	    private GRBVar[][] lambdaVars;
		private ArrayList<GRBVar>[] lambdaVars;
	    
	    // Creating Gurobi constraints
		public GRBConstr[] visitedPickupsCon;
		public GRBConstr[] oneVisitCon; 
		
		// Creating Gurobi objective function
		public GRBLinExpr objective;
		
		// Lists of dual variables
		
		
	    public InstanceData inputdata;
		public float profit = 0;
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
		
		public GurobiInterface(InstanceData inputdata, Vector<Node> deliveryNodes, Vector<Node> pickupNodes, Vector<Node> nodesWithoutDepot, Vector<Vehicle> vehicles, Vector<Float> dualVisitedPickupsCon, Vector<Float> dualOneVisitCon, PrintWriter pw) throws Exception {
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
					System.out.println("vehicleroutes" + vehicles.get(k).vehicleRoutes);
					GRBVar temp = model.addVar(0, GRB.INFINITY, firstLabel.profit, GRB.CONTINUOUS, col, "lambda_"+k+""+r);
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
			
			GRBVar tempVar = model.addVar(0, GRB.INFINITY, l.profit, GRB.CONTINUOUS,  "lambda_"+l.vehicle.number+numberOfRoutes);
			lambdaVars[l.vehicle.number].add(tempVar);
			this.objective.addTerm(l.profit, tempVar);
			
			
			
			for(int i = 0; i < pickupNodes.size(); i++) {
		
				if(l.pickupNodesVisited.contains(pickupNodes.get(i).location)) {
					System.out.println("PICKUP" + i);
					model.chgCoeff(visitedPickupsCon[i], tempVar, 1);
//					System.out.println(visitedPickupsByVehicleOnRoute[l.vehicle.vehicleRoutes.lastElement()][l.vehicle.number][i]);
				}	
			}
				model.chgCoeff(oneVisitCon[l.vehicle.number], tempVar, 1);
			//	System.out.println("Variables"+variables.size());
				
				
				System.out.println("vehicleroutes" + vehicles.get(l.vehicle.number).vehicleRoutes);
				System.out.println(l.path);
				System.out.println("numRoutes"+numberOfRoutes);
				pathList.put(numberOfRoutes-1, l);
				System.out.println(pathList.get(numberOfRoutes-1).profit);
				
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
			Float[] dualVisitedPickupsCon = new Float[pickupNodes.size()];  
			Float[] dualOneVisitCon = new Float[vehicles.size()];
			
			for(int i = 0; i < pickupNodes.size(); i++) {
				float dualPickup_i = (float) visitedPickupsCon[i].get(GRB.DoubleAttr.Pi);
				dualVisitedPickupsCon[i] = dualPickup_i;
				System.out.println("DUAL: " + dualPickup_i);
				//System.out.println("HER");
				//System.out.println(dualVisitedPickupsCon.get(i));
				
			}
//			builder.dualVisitedPickupsCon = dualVisitedPickupsCon;
			
			for(int k = 0; k < vehicles.size(); k++) {
				float dualVehicle_k = (float) oneVisitCon[k].get(GRB.DoubleAttr.Pi);
				dualOneVisitCon[k]=dualVehicle_k;
				System.out.println("DUAL: " + dualVehicle_k);
				//System.out.println("HER");
				//System.out.println(dualOneVisitCon.get(k));
				
			}
//			builder.dualOneVisitCon = dualOneVisitCon;
			
			model.update();
			
			Label bestLabel = new Label();
//			float bestreducedCost = 100000000;
			boolean addedLabel = true;
			System.out.println("Objective value" +model.get(GRB.DoubleAttr.ObjVal));
			int counter = 0;
			while(addedLabel && counter<1000) {
			counter++;
			addedLabel=false;
			for(int k = 0; k < vehicles.size(); k++) {
				System.out.println("vehicle duals: "+Arrays.toString(dualOneVisitCon));
				System.out.println("cargo duals: "+Arrays.toString(dualVisitedPickupsCon));
				Vector<Label> list = builder.BuildPaths(vehicles.get(k), dualVisitedPickupsCon, dualOneVisitCon);
				bestLabel = builder.findBestLabel(list);
				
				if(bestLabel!=null) {
				
					System.out.println("red cost " +bestLabel.reducedCost);
					numberOfRoutes += 1;
					vehicles.get(k).vehicleRoutes.add(numberOfRoutes);
				//	numberOfRoutes += 1;
				//	vehicles.get(l.vehicle.number).vehicleRoutes.add(numberOfRoutes);
					System.out.println ("HER: " +numberOfRoutes);
					addRoute(bestLabel);
					addedLabel=true;
					
				}
			}
				model.optimize();
				model.write("model.lp");
				
//				dualVisitedPickupsCon = new Vector<Float>();  
				
				
				
				for(int i = 0; i < pickupNodes.size(); i++) {
					float dualPickup_i = (float) visitedPickupsCon[i].get(GRB.DoubleAttr.Pi);
					dualVisitedPickupsCon[i] = dualPickup_i;
					System.out.println("DUAL_pickup: " + dualPickup_i);
					//System.out.println("HER");
					//System.out.println(dualVisitedPickupsCon.get(i));
					
				}
//				builder.dualVisitedPickupsCon = dualVisitedPickupsCon;
				for(int k = 0; k < vehicles.size(); k++) {
					float dualVehicle_k = (float) oneVisitCon[k].get(GRB.DoubleAttr.Pi);
					System.out.println("dual of vehicle "+k+": "+dualVehicle_k);
					dualOneVisitCon[k]=dualVehicle_k;
	//				builder.dualOneVisitCon = dualOneVisitCon;
//					System.out.println("DUAL_vehicle: " + dualVehicle_k);

				}
			model.update();
			
				
			for(int k = 0; k < vehicles.size(); k++) {
				int number = 0;
				int routeNumber = 0; 
				
				//int roundCounter = 0;
				//int routeCounter = vehicles.get(k).vehicleRoutes.size()*roundCounter;
				//int routeCounter = 0;
			
				for (GRBVar var : lambdaVars[k]) {
					
					routeNumber = vehicles.get(k).vehicleRoutes.get(number);
					
					number ++;
				//	System.out.print("routenumber" + routeNumber);
				//	System.out.println("routeCouter" +routeNumber);
					if(var.get(GRB.DoubleAttr.X)>0.01 ) {
						System.out.println(var.get(GRB.StringAttr.VarName)  + " " +var.get(GRB.DoubleAttr.X));
				//		int route = lambdaVars[k][r]
						//int route = 2;
						//if((routeCounter%2)!=0 && routeCounter > 2) {
						//	routeCounter++;
						//}
						System.out.println(pathList.get(routeNumber-1).profit);
						
					//	System.out.println("numRoutes"+numberOfRoutes);
						if(routeNumber>vehicles.size()) {
							System.out.println(pathList.get(routeNumber-1).toString());
							//routeNumber++;
							//System.out.println(pathList.get(routeNumber).profit);
						//	System.out.println("routeCouter2" +routeNumber);
							for(int i = 0; i < pathList.get(routeNumber-1).path.size(); i++) {
								System.out.println(pathList.get(routeNumber-1).path.get(i).number);
							}
						}
						
						
				
					}
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