package ColumnGenerator;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Date;
import java.util.Hashtable;
import java.util.PriorityQueue;
import java.util.Vector;

import Objects.Arcbranch;
import Objects.BBNode;
import Objects.Cargo;
import Objects.Filewriter;
import Objects.Path;
import Objects.Port;
import Objects.Ship;
import Objects.TwoCutPath;
import Objects.TwoCutPathComparator;

import com.dashoptimization.*;
import gurobi.*;
import gurobi.GRB.DoubleAttr;
import gurobi.GRB.IntAttr;


import Objects.*;


public class XpressInterface3 {
//	public static void main(String[] args) {
//		//preGeneration();
//		dynamicGen();
//		2
//	}
	
	//private XPRBbasis basis;
	//private XPRS xprs;
	//private XPRM xprm;
	private GRBEnv GurobiEnvironment;
	private GRBModel model;
	private SubproblemSolver sub;
//	private XPRB xprb;
	private RouteBuilder route;
	private RouteBuilder4 route3;
	private RouteBuilder3 route2;
//	private XPRBprob problem;
	private ArrayList<Ship> ships;
	private Hashtable<Integer,Port> ports;
	private ArrayList<Cargo> cargos;
	
	private TreeManager tm;
	private double zeroTol;

	//variables
	private ArrayList<GRBVar> variables;
//	private ArrayList<GRBVar> quantityVars;
	private GRBVar[] optionalVars;
	private GRBVar[] z;
	private GRBVar numberOfCargoes;
	private Vector<Integer> branchedOn; 
	//expressions
//	private GRBLinExpr[] expr2;
//	private GRBLinExpr[] expr;
//	private GRBLinExpr[] Optionalexpr;
//	private GRBLinExpr[] exprcon;
//	private GRBLinExpr numberOfCargoesExpr;
	private GRBVar[][][] branchingslack;
	private GRBVar[][][][] twobranchingslack;
	private GRBVar[] quantitySlack;
	private GRBVar[] slack;
	private GRBVar[] slack2;
	private GRBVar conSlack;
	private GRBLinExpr obj;
//	private GRBLinExpr[] quantity;
//	private GRBLinExpr[] capacity;
	//private GRBLinExpr[] quantityLimit;
	
	//constraints
	private GRBVar numVehicles;
	
//	private GRBConstr[] constraints2;
	private GRBConstr[] constraints;
	private GRBConstr[] optionalcon;
	private GRBConstr convexity;
	private GRBConstr[] quantityCon;
	private GRBConstr[] capacityCon;
	private GRBConstr[][][] arcBranches;
	private GRBConstr[][][][] twoArcBranches;
	private GRBConstr[][] firstarcBranches;
	private GRBConstr numberOfCargoesCon;
	//private GRBConstr[] quantityLimitCon;
	private Hashtable<GRBVar,Integer> allPaths;
	public ArrayList<Path> pathList;
	private BBNode lastnode;
	private Vector<Integer> add;
	private Vector<Integer> remove;
	//ArrayList<Path>[] shipspaths; 
//	private Filewriter file;
	private double timeUsed1;
	private double timeUsed2;
	private double timeUsed3;
	
	
	private GRBLinExpr[][][] arcExpr;
	private TwoPathCuts two;
	private CapacityCuts capcuts;
	private ArrayList<Vector<Integer>> twopathcutsSubsets;
	private Hashtable<Vector<Integer>, GRBConstr> cutConstraints;
//	private Hashtable<Vector<Integer>, GRBLinExpr> cutexpr;
	private ArrayList<Vector<Integer>> capcutsSubsets;
	private Hashtable<Vector<Integer>, GRBConstr> capcutConstraints;
//	private Hashtable<GRBVar, GRBVar[]> qVars;
	
	
	public GRBConstr[][][] shipCargoCuts;
//	public GRBConstr[][] nonOverFullGroupCuts;
//	public GRBConstr[] nonOverFullGroupCuts2;
	public GRBConstr[] nonOverFullGroupCuts3;
//	public GRBConstr[][] overFullGroupCuts;
//	public GRBConstr[][] overFullGroupCuts2;
	
	private GRBConstr[][][] subsetRowCuts;
//	private Hashtable<Vector<Integer>, GRBLinExpr> capcutexpr;
//	private double[][] arcdualvalues;
	//private Ship supership;
	
	public XpressInterface3(ArrayList<Ship> ships, Hashtable<Integer,Port> ports, ArrayList<Cargo> cargos, double zeroTol, TreeManager tm) throws Exception{
		
//		try {
			this.GurobiEnvironment = new GRBEnv();
			GurobiEnvironment.set(GRB.IntParam.OutputFlag, 0);
			GurobiEnvironment.set(GRB.IntParam.Presolve, 0);
			GurobiEnvironment.set(GRB.DoubleParam.OptimalityTol, 0.000000001);
			model = new GRBModel(this.GurobiEnvironment);
			
			sub = new SubproblemSolver(GurobiEnvironment, cargos, tm);
			
//		}
//		catch(Exception e) {
//			e.printStackTrace();
//		}
//		
		//checkCon = new GRBConstr[cargos.size()];
		//this.shipspaths = new ArrayList[ships.size()]; 
		//this.supership = Preprocess.makeSuperShip(ships, cargos);
//		this.overFullGroupCuts = new GRBConstr[cargos.size()][cargos.size()];
//		this.overFullGroupCuts2 = new GRBConstr[cargos.size()][cargos.size()];
//		this.nonOverFullGroupCuts = new GRBConstr[ships.size()][cargos.size()];
//		this.nonOverFullGroupCuts2 = new GRBConstr[cargos.size()];
    	
		this.shipCargoCuts = new GRBConstr[ships.size()][cargos.size()][cargos.size()];
		this.twopathcutsSubsets = new ArrayList<Vector<Integer>>();
		this.cutConstraints = new Hashtable<Vector<Integer>, GRBConstr>();
//		this.cutexpr = new Hashtable<Vector<Integer>, GRBLinExpr>();
		this.capcutsSubsets = new ArrayList<Vector<Integer>>();
		this.capcutConstraints = new Hashtable<Vector<Integer>, GRBConstr>();
//		this.capcutexpr = new Hashtable<Vector<Integer>, GRBLinExpr>();
		this.lastnode = null;
		this.add = new Vector<Integer>();
		this.remove = new Vector<Integer>();
		this.tm=tm;
		this.route = new RouteBuilder(cargos,ports,tm, ships);
		this.route3 = new RouteBuilder4(cargos,ports,tm, ships);
		this.route3.setSubproblemSOlver(sub);
		this.route2 = new RouteBuilder3(cargos,ports,tm, ships);
		//this.route.buildIncompatibilityMatrix(ships.get(0));
		//this.xprs = new XPRS();
		//this.xprm = new XPRM();
		//xprs.init();
//		this.file = new Filewriter("nodeinfo.txt");
//		this.xprb = new XPRB();
		this.ships = ships;
		this.cargos = cargos;
		this.ports = ports;
		this.pathList = new ArrayList<Path>();
		this.zeroTol = zeroTol;
		two = new TwoPathCuts(cargos, ports, tm, ships);
		capcuts = new CapacityCuts(cargos, ports, tm, ships);
		this.arcBranches = new GRBConstr[2][cargos.size()*2][cargos.size()*2+1];
		this.twoArcBranches = new GRBConstr[2][cargos.size()*2][cargos.size()*2][cargos.size()*2+1];
		this.firstarcBranches = new GRBConstr[2][cargos.size()];
		for(int s = 0; s < ships.size(); s++) {
//			ColumnPool.add(new Vector<Integer>());
//			exprcon[s] = new GRBLinExpr();
			ships.get(s).setOriginalVolumeCapacity(ships.get(s).getVolumeCapacity());
		}
		
		this.arcExpr = new GRBLinExpr[2][cargos.size()*2][cargos.size()*2+1];
		for(int i = 0; i < cargos.size()*2; i++) {
			for(int j = 0; j < cargos.size()*2+1; j++) {
				arcExpr[0][i][j] = new GRBLinExpr();
				arcExpr[1][i][j] = new GRBLinExpr();
			}
		}
		this.timeUsed1 = 0;
		this.timeUsed2 = 0;
		this.timeUsed3 = 0;
		this.variables = new ArrayList<GRBVar>();
		this.allPaths = new Hashtable<GRBVar, Integer>();
//		this.qVars = new Hashtable<GRBVar, GRBVar[]>();
		initiateProblem();
	}
	
	public BBNode[] dynamicGen(BBNode node) throws Exception{
	
		Date date1 = new Date();

		solveProblem(node);
		
//		delSubsetRowCuts();
		
		if(nonOverFullGroupCuts3!=null) {
			for(int i = 0; i <cargos.size(); i++) {
				if(nonOverFullGroupCuts3[i]!=null) {
					model.remove(nonOverFullGroupCuts3[i]);
				}
			}
		}
		nonOverFullGroupCuts3 = null;
		
		for(int i = 0; i < cargos.size(); i++) {
			if(firstarcBranches[0][i]!=null) {
				model.remove(firstarcBranches[0][i]);
				firstarcBranches[0][i]=null;
			}
			if(firstarcBranches[1][i]!=null) {
				model.remove(firstarcBranches[1][i]);
				firstarcBranches[1][i]=null;
			}
		}
		
		for(int i = 0; i < cargos.size()*2; i++) {
			for(int j = 0; j < cargos.size()*2+1; j++) {
				if(arcBranches[0][i][j]!=null) {
					arcExpr[0][i][j] = model.getRow(arcBranches[0][i][j]);
//					double rhs = arcExpr[0][i][j].getConstant();
////					System.out.println("Before "+arcExpr[0][i][j].getConstant());
//					arcExpr[0][i][j].addConstant((-1*rhs));
//					System.out.println("rhs of con "+arcBranches[0][i][j].get(GRB.DoubleAttr.RHS));
//					System.out.println("size: "+arcExpr[0][i][j].size());
//					System.out.println("After "+arcExpr[0][i][j].getConstant());
//					System.exit(0);
					model.remove(arcBranches[0][i][j]);
					arcBranches[0][i][j]=null;
				}
				if(arcBranches[1][i][j]!=null) {
					arcExpr[1][i][j] = model.getRow(arcBranches[1][i][j]);
//					System.out.println("Before "+arcExpr[1][i][j].getConstant());
//					double rhs = arcExpr[1][i][j].getConstant();
//					arcExpr[1][i][j].addConstant((-1*rhs));
//					System.out.println("After "+arcExpr[1][i][j].getConstant());
//					System.out.println("rhs of con "+arcBranches[1][i][j].get(GRB.DoubleAttr.RHS));
//					System.out.println("size: "+arcExpr[1][i][j].size());
//					System.exit(0);
					model.remove(arcBranches[1][i][j]);
					arcBranches[1][i][j]=null;
				}
				
			}
		}
		for(int i = 0; i < cargos.size()*2; i++) {
			for(int j = 0; j < cargos.size()*2; j++) {
				for(int k = 0; k < cargos.size()*2+1; k++) {
					if(twoArcBranches[0][i][j][k]!=null) {
						model.remove(twoArcBranches[0][i][j][k]);
						twoArcBranches[0][i][j][k]=null;
					}
					if(twoArcBranches[1][i][j][k]!=null) {
						model.remove(twoArcBranches[1][i][j][k]);
						twoArcBranches[1][i][j][k]=null;
					}
				}
			}
		}
		//model.update();
//		for(int i = 0; i <cargos.size(); i++) {
//			for(int j = i+1; j <cargos.size(); j++) {
//				if(overFullGroupCuts[i][j]!=null) {
//					problem.delCtr(overFullGroupCuts[i][j]);
//					overFullGroupCuts[i][j]=null;
//				}
//				if(overFullGroupCuts2[i][j]!=null) {
//					problem.delCtr(overFullGroupCuts2[i][j]);
//					overFullGroupCuts2[i][j]=null;
//				}
//			}
//		}
//		XPRS xprs = new XPRS();
//		XPRM xprm = new XPRM();
//		xprs.init();
//		System.out.println(xprs.getVersion());
//		XPRB xprb = new XPRB();
//		XPRBprob problem = xprb.newProb("problem 1");
		
		
		lastnode = node;
		
//		file.writeTestOutput("Is solved: "+node.isSolved());
//		file.writeTestOutput("NOde id: "+node.getNodeId());
//		file.writeTestOutput("Objective Value: "+node.getObjectiveValue());
//		file.writeTestOutput("number of cargoes upperbound "+node.getNumberOfCargoesBranchUpper());
//		file.writeTestOutput("number of cargoes lowerbound "+node.getNumberOfCargoesBranchLower());
//		for(int b = 0; b < node.getBranches().size(); b++) {
//			file.writeTestOutput("Branching on: "+tm.getBranches().get(node.getBranches().get(b)).cargo+"  "+tm.getBranches().get(node.getBranches().get(b)).decision);
//		}
//		for(int b = 0; b < node.getCargoBranches().size(); b++) {
//			file.writeTestOutput("Branching on: "+tm.getCargoBranches().get(node.getCargoBranches().get(b)).getCargo()+"   "+tm.getCargoBranches().get(node.getCargoBranches().get(b)).getValue()+"  "+tm.getCargoBranches().get(node.getCargoBranches().get(b)).getDecision());
//		}
//		file.writeTestOutput("Node depth: "+node.getDepth());
//		
//		file.writeTestOutput("-------------------");
//		file.flush();
		//for(int i = 0 ; i < ships.size(); i++) {
		//	System.out.println("Ship number "+i+" has "+shipspaths[i].size());
		//}
		
		if(node.getObjectiveValue()<tm.LowerBound && node.isSolved()) {
//			file.writeTestOutput("-------------------------------");
//			file.writeTestOutput("Node pruned");
//			file.writeTestOutput("-------------------------------");
			return null;
		}
//		file.writeTestOutput("Quantity:");
//		for(GRBVar q : quantityVars) {
//			if(q.get(GRB.DoubleAttr.X)>tm.getZeroTol()) {
//				file.writeTestOutput(q.getName()+" got solution "+q.get(GRB.DoubleAttr.X));
//				//System.out.println(q.getName()+" got solution "+q.get(GRB.DoubleAttr.X));
//			}
//		}
		//lastnode = node;
		
//		file.writeTestOutput("-------------------------------");
//		file.writeTestOutput("Adding: "+add.toString());
//		file.writeTestOutput("Removing: "+remove.toString());
		ArrayList<Integer> nonBinaryVariables = new ArrayList<Integer>();
		ArrayList<Integer> nonZeroVariables = new ArrayList<Integer>();
		ArrayList<Path> solution = new ArrayList<Path>();
		Hashtable<Path, Double> solutionvars = new Hashtable<Path, Double>();
		System.out.println(variables.size());
		for(int i = 0; i < variables.size(); i++) {
			if(variables.get(i).get(GRB.DoubleAttr.X)>zeroTol) {
//				System.out.println(variables.get(i).get(GRB.DoubleAttr.X)+ "     "+pathList.get(allPaths.get(variables.get(i))).toString());
				if(ships.indexOf(pathList.get(allPaths.get(variables.get(i))).ship) <0 && node.isSolved()) {
//					if(node.isSolved()==true) {
						node.setObjectiveValue(-9999999);
						System.out.println("problem lies here");
//						System.out.println(variables.get(i).get(GRB.DoubleAttr.X));
//						System.out.append(pathList.get(allPaths.get(variables.get(i))).toString());
//						System.exit(0);
//					}
					return null;
				}
				//file.writeTestOutput("Number of Variables: "+variables.size());
				//file.writeTestOutput(variables.get(i).getName()+" "+variables.get(i).get(GRB.DoubleAttr.X)+" "+ i);
//				file.writeTestOutput(pathList.get(allPaths.get(variables.get(i))).toString2() +" "+variables.get(i).get(GRB.DoubleAttr.X));
				if(ships.indexOf(pathList.get(allPaths.get(variables.get(i))).ship) >=0) {
					nonZeroVariables.add(i);
				}
				//System.out.println(variables.get(i).getName()+" "+variables.get(i).get(GRB.DoubleAttr.X)+" "+ i);
				//System.out.println(allPaths.get(variables.get(i)).toString());
				if(variables.get(i).get(GRB.DoubleAttr.X)<1-zeroTol) {
					nonBinaryVariables.add(i);
				}
				//else {
					solution.add(pathList.get(allPaths.get(variables.get(i))));
					solutionvars.put(pathList.get(allPaths.get(variables.get(i))), variables.get(i).get(GRB.DoubleAttr.X));
				//}
			}
		}
		
		node.setSolution(solution);
		node.setSolutionvars(solutionvars);
//		file.writeTestOutput("-------------------------------");
//		file.flush();
		
//		BBNode[] temp = branchOnNumberOfCargo(node);
//		if(temp!= null) {
//			return temp;
//		}
//		temp = branchOnCargo(nonZeroVariables, node);
//		if(temp!= null) {
//			return temp;
//		}
//		else {
//			return branchOnCargoAndShip(nonBinaryVariables, node);
//		double[] yvars = new double[cargos.size()*2];
//		for(int i = 0; i < cargos.size(); i++) {
//			yvars[i] = optionalVars[i].get(GRB.DoubleAttr.X);
//			yvars[i+cargos.size()] = optionalVars[i].get(GRB.DoubleAttr.X);
//		}
//		
//		double[][] arcflow = getArcFlow(nonZeroVariables);
//		
//		generateTwoPathCuts(arcflow, yvars, nonZeroVariables);
//		for(int i = 0; i<cargos.size()*2; i++) {
//			for(int j = 0; j<cargos.size()*2+1; j++) {
//				if(arcflow[i][j]>0) {
////					System.out.println(i+" "+j+" "+arcflow[i][j]);
//				}
//			}
//		}
//		int counter = generateTwoPathCuts(arcflow, yvars, two);
//		System.out.println("number of cuts generated "+counter);
//		return selectBranching(nonZeroVariables, node);
//		}
		//file.writeTestOutput("COunter 2: "+counter2);
		
		
		BBNode[] nodes = selectBranching(nonZeroVariables, node);
		if(nodes == null) {
			for(int i = 0; i < cargos.size(); i++) {
				System.out.println("y "+i+" "+optionalVars[i].get(GRB.DoubleAttr.X));
				System.out.println("z "+i+" "+z[i].get(GRB.DoubleAttr.X));
			}
			//for(GRBVar q : quantityVars) {
				//if(q.get(GRB.DoubleAttr.X)>tm.getZeroTol()) {
					//file.writeTestOutput(q.getName()+" got solution "+q.get(GRB.DoubleAttr.X));
					//System.out.println(q.getName()+" got solution "+q.get(GRB.DoubleAttr.X));
				//}
			//}
		
			//System.out.println("IP sol "+ problem.getObjVal());
			node.setIPsol(model.get(GRB.DoubleAttr.ObjVal));
		}
		return nodes;		
	}
		
//	
	
//	private void findAlteredVariables(BBNode current, BBNode last) {
//		BBNode currenttree = current;
//		BBNode lasttree = last;
//		this.add = new Vector<Integer>();
//		this.remove = new Vector<Integer>();
//		if(currenttree!=null && lasttree!=null) {
//		
//		while(currenttree.getNodeId() != lasttree.getNodeId()) {
//			if(currenttree.getDepth()<lasttree.getDepth()) {
//				add.addAll(lasttree.getBranchVariables());
//				//remove.addAll(lasttree.getGeneratedVariables());
//				lasttree=lasttree.getParent();
//			}
//			else {
//				remove.addAll(currenttree.getBranchVariables());
//				//add.addAll(currenttree.getGeneratedVariables());
//				currenttree=currenttree.getParent();
//			}
//		}
//		
//	}
//	}
	
	private void solveProblem(BBNode node) throws Exception{
//		if(problem!=null) {
//			problem.reset();
//		}
//		initiateProblem();
		ArrayList<Integer> nonZeroVariables = new ArrayList<Integer>();
		
	System.out.println("solving problem");
		
 		XPRBbasis basis = node.getBasis();
		int[] shipsPri = new int[ships.size()];
		for(int i = 0; i < ships.size();i++) {
			shipsPri[i] = i;
			
			ships.get(i).setVolumeCapacity(999999999);
			ships.get(i).setDual(0);
		}
		
		if(!fixBranching(node)) {
			return;
		}

		
		int zeropathscounter = 0;
		int counter2 = 0;
		int counter = 3;
		for(int i = 0; i <cargos.size(); i++) {
			cargos.get(i).setDual(-1*cargos.get(i).getIncome());
			cargos.get(i).setQuantityDual(0);
			cargos.get(i).setDual2(0);
			//System.out.println(constraints[i].getDual());
			//constraints[i].print();
		}
		
		ArrayList<Path> optimalPaths = new ArrayList<Path>();
		double ObjectiveValue = 0;
		int ObjCounter=0;
		boolean solveSub = true;
		setUpperAndLowerBound(node);
		int subproblem = 0;
//		if(node.getDepth()==0 && variables.size()==0) {
//			subproblem=1;
//		}
		boolean addedcuts = false;
		route.initiateSubproblem(node);
		route2.initiateSubproblem(node);
		route3.initiateSubproblem(node);
		addSubsetRowCuts();
		
		jumppoint:
		while( counter2<tm.MaxIter){
//			System.out.println("counter2 is now: "+counter2+ " and subproblem is "+subproblem);
			double[] reducedCost = new double[ships.size()];
			Arrays.fill(reducedCost, 999999999);
			//double reducedCost = 0;
			counter = 0;
			
			//if(node.getDepth()==0 || counter2>1) {
			if(solveSub) {
				
				ArrayList<Path> superPaths = null;
//				if(subproblem == 4) {
////				superPaths = route.buildSuperPaths(supership, cargos, ports, node, tm, cargos.size());
//				//System.out.println("number of superPaths "+superPaths.size());
//				
//				}
			
			for(int s = 0; s < ships.size(); s++) {
//				System.out.println("subproblem: "+subproblem);
//				System.out.println(ships.get(shipsPri[s]%ships.size()).getName());
				Date date1 = new Date();
				boolean notnew = true;
				ArrayList<Path> paths = new ArrayList<Path>();
				//ArrayList<Path> paths2 = null;
				if(subproblem==0) {
////				//if(counter2==1) {
					Date dateUnused1 = new Date();
//					paths = mainColumns(ColumnPool.get(s), node);
					if(node.getDepth()==0 && counter2==0 && variables.size()==0) {
						paths = route.buildArtificialPaths(cargos, ports, node, tm);
//						for(int s2 = 0; s2 < ships.size(); s2++) {
//						paths.addAll(route.buildPaths(ships.get(s2), cargos, ports, node, tm));
//						}
						notnew= false;
						
					}
					else if(counter2>0){
						//paths = new ArrayList<Path>();
						paths = generateNewDelPatterns();
						//paths = new ArrayList<Path>();
						notnew = false;
					}
					else {
						paths = new ArrayList<Path>();
						counter = 1;
					}					
					Date dateUnused2 = new Date();
					this.timeUsed1 += dateUnused2.getTime()-dateUnused1.getTime();
				}
				//if(paths==null || paths.isEmpty()) {
					if(subproblem==1) {
					Date dateUnused3 = new Date();
					
					//paths = route.buildHeurPaths(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, optimalPaths);
					//paths = route.buildNonPaths5(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, cargos.size());
					
					paths = route2.buildNonPaths4(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, (int)(cargos.size()));
					
					// This one is correct
//					paths = route.buildNonPaths5(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, (int)(cargos.size()/2));
					
					
					
					//paths = route.buildNonPaths6(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, cargos.size(), 3, 1);
//					if(node.getNodeId()==0 && counter2==0) {
//					paths.addAll(route.buildPaths(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm));
//					}
//					if(counter2==1 && node.getDepth()==0) {
//						paths.addAll(route.buildPaths(ships.get(s), cargos, ports, node, tm));
//					}
					tm.pathcounter1+=paths.size();
					tm.subproblemcounter1++;
					Date dateUnused4 = new Date();
					this.timeUsed2 += dateUnused4.getTime()-dateUnused3.getTime();
					notnew = false;
				}
					if(subproblem==2) {
						Date dateUnused3 = new Date();
						//paths = route.buildHeurPaths(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, optimalPaths);
						
						//paths = route.buildNonPaths6(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, (int) (cargos.size()/2), 4);
						
						
						//this one is correct
					//	paths = route.buildNonPaths5(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, cargos.size());
						paths = route3.buildNonPaths5(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, (int)(cargos.size()*1/2));
						
						
						
						
						//paths = route.buildNonPaths6(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, cargos.size(), 3, 2);
//						if(cargos.size()>10) {
							//paths = route.buildNonPaths5(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, (int)(cargos.size()*3/4));
//						}
//						else {
//						paths = new ArrayList<Path>();
//						}
						tm.pathcounter2+=paths.size();
						tm.subproblemcounter2++;
						Date dateUnused4 = new Date();
						this.timeUsed2 += dateUnused4.getTime()-dateUnused3.getTime();
						notnew = false;
//						if(!paths.isEmpty()) {
//							System.out.println(paths.get(0).toString());
//						}
					}
				//if(paths==null || paths.isEmpty()) {
				  if(subproblem == 3) {
//					//if(counter2==1) {
					notnew = false;
//					Date dateUnused5 = new Date();
					//paths = route.buildNonPaths6(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, (int) (cargos.size()), 4);
					//paths = route.buildNonPaths6(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, cargos.size(), 3, 3);
					
					
					paths = route3.buildNonPaths5(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, (int)(cargos.size()));
					
					
					
					tm.pathcounter3+=paths.size();
					tm.subproblemcounter3++;
					//Date dateUnused6 = new Date();
//					//this.timeUsed3 += dateUnused6.getTime()-dateUnused5.getTime();
//					//}
////					else {
//						//paths2 = route.buildNonPaths3(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm);
////					}
				}
				//if(paths==null || paths.isEmpty()) {
				  if(subproblem == 4) {
//					//if(counter2==1) {
					notnew = false;
				//	paths = route.buildNonPaths6(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, (int) (cargos.size()/3), 5);
//					Date dateUnused5 = new Date();
					
					
					paths = route3.buildNonPaths4(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, (int)(cargos.size()*1/2));
					
					
					tm.pathcounter4+=paths.size();
					tm.subproblemcounter4++;
					//Date dateUnused6 = new Date();
//					//this.timeUsed3 += dateUnused6.getTime()-dateUnused5.getTime();
//					//}
////					else {
//						//paths2 = route.buildNonPaths3(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm);
////					}
				}
				  if(subproblem == 5) {
//						//if(counter2==1) {
						notnew = false;
						Date dateUnused5 = new Date();
//						if(cargos.size()>10) {
						//paths = route.buildNonPaths6(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, cargos.size(), 4, cargos.size());
							paths = route3.buildNonPaths4(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, (int)(cargos.size()*3/4));
							//paths = route.buildNonPaths4(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, 10);
						Date dateUnused6 = new Date();
						//tm.totalTimeIn6 += dateUnused6.getTime()-dateUnused5.getTime();
//						}
//						else {
//						paths = new ArrayList<Path>();
//						}

						
						
						tm.pathcounter5+=paths.size();
						tm.subproblemcounter5++;
						//Date dateUnused6 = new Date();
//						//this.timeUsed3 += dateUnused6.getTime()-dateUnused5.getTime();
//						//}
////						else {
//							//paths2 = route.buildNonPaths3(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm);
////						}
					}
				//ArrayList<Path> paths2 = null;
				//if(paths==null || paths.isEmpty()) {
				  if(subproblem==8 ) {
					  Date cuttime1 = new Date();
					  
					  if(node.getDepth()<=-4 && !addedcuts) {
						  addedcuts=true;
					  double[] yvars = new double[cargos.size()*2];
						for(int i = 0; i < cargos.size(); i++) {
							yvars[i] = optionalVars[i].get(GRB.DoubleAttr.X);
							yvars[i+cargos.size()] = optionalVars[i].get(GRB.DoubleAttr.X);
						}
					  
						
						double[][] arcflow = getArcFlow(nonZeroVariables);
						boolean isfractional =checkFractionalFlow(arcflow);
						two.setArcflow(arcflow);
						two.setYvars(yvars);
						if(isfractional && ObjCounter-tm.getZeroTol()>tm.LowerBound) {
							counter = generateTwoPathCuts(nonZeroVariables);
						}
						else {
							counter = 0;
						}
						System.out.println("number of two path cuts generated "+counter);
						
					}
//					for(int i = 0; i < cargos.size(); i++) {
//						constraints2[i].setType(XPRB.G);
//					}
//					counter=2; 
					Date cuttime2 = new Date();
					tm.totalTimeInCuts += (cuttime2.getTime()-cuttime1.getTime())/1000;
//						counter += generateCapacityCuts(arcflow, yvars, capcuts);
//						if(counter==0 && !solveSub) {
//							solveSub=true;
//							subproblem=0;
//							
//						}
//					  }
//					  notnew = false;
					  paths = new ArrayList<Path>();

					  tm.pathcounter5+=paths.size();
				  }
				  
				  if(subproblem == 7) {
					//if(counter2==1) {
					notnew = false;
					Date dateUnused5 = new Date();
					//paths = route.buildNonPaths4(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, (int) cargos.size());
					Date dateUnused6 = new Date();
					tm.Rbtime+=dateUnused6.getTime()-dateUnused5.getTime();
					paths = route3.buildNonPaths4(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, (int) cargos.size());
					Date dateUnused7 = new Date();
					tm.Rb4time+=dateUnused7.getTime()-dateUnused6.getTime();
					//paths = route.buildNonPaths6(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm, (int) (cargos.size()), 5);
					//System.out.println(paths.size());
					
					
//					compareTwoPaths(paths, paths2);
					
//					if(tm.Rb4paths>tm.Rbpaths) {
//						System.exit(0);
//					}
					
//					tm.totalTimeIn4 += dateUnused6.getTime()-dateUnused5.getTime();
					tm.pathcounter6+=paths.size();
					tm.subproblemcounter6++;
					if(!paths.isEmpty()) {
//						System.out.println(paths.get(0).getDualLowerBound());
//						System.out.println(paths.get(0).toString());
						reducedCost[shipsPri[s]%ships.size()]= paths.get(0).getDualLowerBound();
					}
					else {
						zeropathscounter++;
						reducedCost[shipsPri[s]%ships.size()]=0;
					}
					//}
//					else {
						//paths2 = route.buildNonPaths3(ships.get(shipsPri[s]%ships.size()), cargos, ports, node, tm);
//					}
				}
				
				
				counter += paths.size();
				Date date8 = new Date();
				tm.totalTimeInSub1 += (date8.getTime()-date1.getTime());
				for(int i = 0; i <paths.size(); i++) {
					
					comparePaths(paths.get(i), node);
					
//					System.out.println(paths.get(i).toString());
					
					GRBVar temp = model.addVar(0, 1000, 0, GRB.CONTINUOUS, paths.get(i).ship.getName()+","+variables.size());
					model.update();
					
					if(!notnew) {

						pathList.add(paths.get(i));
						
						allPaths.put(temp, pathList.size()-1);
					}
					else {
						
						allPaths.put(temp, pathList.indexOf(paths.get(i)));
					}
					variables.add(0, temp);
					
					int[] count = new int[cargos.size()];
					//ColumnPool.get(s).add(variables.size()-1);
					for(int j : paths.get(i).cargosHandled) {
						count[j]++;
						//expr[j].addTerm(temp, Math.min(1,paths.get(i).getCargoesHandled()[j]));
						model.chgCoeff(constraints[j],temp, count[j]);
//						model.chgCoeff(constraints2[j], temp, 1);
//						constraints2[j].addTerm(temp, Math.min(2, Math.ceil(paths.get(i).ship.getOriginalVolumeCapacity()/(cargos.get(j).getQuantity()-0.001))));
						//expr[j].addTerm(temp, 1);
					}
//					for(int j = 0; j < cargos.size(); j++) {
//						if(paths.get(i).cargosHandled.contains(j)) {
//							model.chgCoeff(constraints2[j],temp,1);
//						}
//					}
					
					if(ships.indexOf(paths.get(i).ship)>-1) {
						model.chgCoeff(convexity,temp, 1);
					}
					obj.addTerm(paths.get(i).getRevenue(), temp);
				//	ArrayList<Vector<Integer>> delGroups = getDeliveryGroups(paths.get(i));
					//GRBLinExpr[] tempCapexpr = new GRBLinExpr[delGroups.size()];
//					GRBVar[] qvar = new GRBVar[cargos.size()];
//					qVars.put(temp, qvar);
//					boolean check = false;
					Vector<Integer> tempVec = new Vector<Integer>();
					for(int k : paths.get(i).cargosHandled) {
							if(!tempVec.contains(k)) {
							model.chgCoeff(quantityCon[k],temp, paths.get(i).cargoVolume[k]);
							tempVec.add(k);
//								if(paths.get(i).cargoVolume[k]>cargos.get(k).getQuantity()) {
//									System.out.println(paths.get(i).toString());
//									System.out.println(paths.get(i).cargosHandled);
//									System.out.println(Arrays.toString(paths.get(i).cargoVolume));
//									
//									check = true;
									
//								}
							}

					}
//					if(check) {
//						quantityCon[15].print();
//						System.exit(0);
//					}
					
				
					
					for(int m = 0; m < 2; m++) {
						if(paths.get(i).getPortList().get(1).getId()<cargos.size()) {
							if(firstarcBranches[m][paths.get(i).getPortList().get(1).getId()]!=null) {
								model.chgCoeff(firstarcBranches[m][paths.get(i).getPortList().get(1).getId()],temp, 1);
							}
						}
//						for(int k = 0; k < cargos.size()*2; k++) {
//							for(int l = 0; l < cargos.size()*2+1; l++) {
//								if(arcBranches[m][k][l]!= null) {
									int previous = -1;
									int current = paths.get(i).getPortList().get(1).getId();
//									int tempCounter = 0;
									for(int j = 2; j < paths.get(i).getPortList().size(); j++) {
										previous = current;
										current = paths.get(i).getPortList().get(j).getId();
//										if(previous == k && current == l) {
										if(arcBranches[m][previous][current]!= null) {
											double tempCounter = model.getCoeff(arcBranches[m][previous][current], temp);
											tempCounter+=1;
											model.chgCoeff(arcBranches[m][previous][current],temp, tempCounter);
											model.update();
										}
										else {
//											double tempCounter = arcExpr[m][previous][current].get;
//											tempCounter+=1;
											arcExpr[m][previous][current].addTerm(1, temp);
										}
//										}
									}
//								if(arcBranches[m][k][l]!= null) {
//									model.chgCoeff(arcBranches[m][k][l],temp, tempCounter);
//								}
								
//							}
//						}
						if(paths.get(i).getPortList().size()>3) {
							for(int k = 0; k < cargos.size()*2; k++) {
								for(int l = 0; l < cargos.size()*2; l++) {
									for(int n = 0; n < cargos.size()*2+1; n++) {
										if(twoArcBranches[m][k][l][n]!= null) {
											previous = paths.get(i).getPortList().get(1).getId();
											current = paths.get(i).getPortList().get(2).getId();
											int next = -1;
											int tempCounter = 0;
											for(int j = 3; j < paths.get(i).getPortList().size(); j++) {
												next = paths.get(i).getPortList().get(j).getId();
												if(previous == k && current == l && next ==n) {
													tempCounter++;
													
												}
												previous = current;
												current = next;
											}
											model.chgCoeff(twoArcBranches[m][k][l][n],temp, tempCounter);
										}
									}
								}
							}
						}
					}
					addPathToCuts(temp, paths.get(i));

				}
	
				Date date2 = new Date();
				tm.totalTimeInSub += (date2.getTime()-date1.getTime());
				tm.totalTimeInSub2 += (date2.getTime()-date8.getTime());
				if(paths.size()>0 && subproblem==7) {
					for(int i = 0; i < ships.size(); i++) {
						shipsPri[i]++;
					}
					break;
				}
				if(subproblem==6 || subproblem==8 || subproblem==0) {
					break;
				}
			}
			
//			if(counter2>1) {
//				counter = addQuantityCuts(variables, qVars);
//			}
			
//			}
//			else {
//				counter = 1;
//			}
			
			
		//	System.out.println("Time of Sub: "+tid);
			//System.out.println("Generating "+counter+" number of paths");
			}
			
		
			model.setObjective(obj);
			
			if(counter==0) {
//				System.out.println("No paths generated with subproblem "+subproblem);

				if(subproblem==8) {
					break;
				}
//				if(subproblem==6 && addedcuts && problem.getObjVal()>tm.LowerBound) {
//					break;
//				}
				//zeropathscounter++;
				if(subproblem==7 && zeropathscounter==ships.size()) {
					subproblem++;
					node.setUpperbound(model.get(GRB.DoubleAttr.ObjVal));

				}
				
				else if(subproblem<7){
					subproblem++;
				}

			}
			
			//System.out.println("time in sub "+(date2.getTime()-date1.getTime()));
			if(counter>0) {
				Date date4 = new Date();
//				System.out.println("Iteration: "+counter2+" subproblem: "+subproblem+" # of paths "+counter);
				zeropathscounter=0;
//				if(subproblem==7) {
//					double redcost = 0;
//					for(int s = 0; s < ships.size(); s++) {
//						redcost += reducedCost[s];
//					}
//					if(ObjectiveValue+redcost<tm.LowerBound) {
//						for(int i = 0; i < variables.size(); i++) {
//							int p = allPaths.get(variables.get(i));
////							ColumnPool.get(ships.indexOf(pathList.get(p).ship)).add(p);
//						}
//						node.setObjectiveValue(-9999999);
//						node.setSolved(true);
//						return;
//					}
//				}
			subproblem = 0;
			counter2++;
//			problem.loadMat();
//			if(basis!=null) {
//				problem.loadBasis(basis);
//			}

//			problem.print();
//			System.out.println(problem.getVarByName("z_1").toString());
			//problem.maxim();
//			problem.setSense(XPRB.MAXIM);
//			problem.sync(XPRB.XPRS_PROB);
			model.sync();
//			if(counter2==1) {
//				problem.solve("ld");
//			}
//			else {
//				problem.solve("lp");
//			}
//			problem.solve("l");
			model.set(IntAttr.ModelSense, GRB.MAXIMIZE);
			model.optimize();
		//	System.out.println("solves problem");
			if(model.get(IntAttr.Status)==GRB.INFEASIBLE) {
				
				
				System.out.println("infeasible problem");
				System.exit(0);
//				for(int i = 0; i < variables.size(); i++) {
					for(int i = 0; i < cargos.size(); i++) {
						System.out.println("z "+i+" LB "+ z[i].get(GRB.DoubleAttr.LB));
						System.out.println("z "+i+" UB "+ z[i].get(GRB.DoubleAttr.UB));
						System.out.println("y "+i+" LB "+ optionalVars[i].get(GRB.DoubleAttr.LB));
						System.out.println("y "+i+" UB "+ optionalVars[i].get(GRB.DoubleAttr.UB));
//					int p = allPaths.get(variables.get(i));
//					ColumnPool.get(ships.indexOf(pathList.get(p).ship)).add(p);
				}
				Date date3 = new Date();
				tm.totalTimeInMaster += (date3.getTime()-date4.getTime());
				node.setObjectiveValue(-99999999);
				node.setSolved(true);

				return;
			}
			//problem.print();
//			basis = problem.saveBasis();
			optimalPaths.clear();
//			if(problem.getObjVal()<0 && node.getDepth()>0 && counter2>1) {
//				node.setObjectiveValue(-1000);
//				node.setSolved(true);
//				return;
//			}
			if(ObjectiveValue-zeroTol<model.get(DoubleAttr.ObjVal)&& ObjectiveValue+zeroTol>model.get(DoubleAttr.ObjVal)){
//				System.out.println("Objective counter: "+ObjCounter+ " added cuts " +addedcuts);
				ObjCounter++;
//				if(ObjCounter>=4 && ObjectiveValue>tm.LowerBound && !addedcuts){
//					counter2=tm.MaxIter;
//				}
			}
			else {
				ObjectiveValue=model.get(DoubleAttr.ObjVal);
				ObjCounter = 0;
			}
			nonZeroVariables.clear();
			for(int i = 0; i < variables.size(); i++) {
				if(variables.get(i).get(GRB.DoubleAttr.X)>=zeroTol) {
					nonZeroVariables.add(i);
//					System.out.print(variables.get(i).get(GRB.DoubleAttr.X)+"    ");
//					System.out.println(pathList.get(allPaths.get(variables.get(i))).toString());
//					optimalPaths.add(pathList.get(allPaths.get(variables.get(i))));
				}
			}
			
			removeVariables();
			setArcDualValues();
			setShipCargoDuals();
//			setSubsetRowDuals();
			ships.get(0).setDual(convexity.get(GRB.DoubleAttr.Pi));
			System.out.println("Iteration number "+counter2 +" with objective "+model.get(DoubleAttr.ObjVal));
			for(int i = 0; i <cargos.size(); i++) {
//				System.out.println(i+" "+optionalVars[i].get(GRB.DoubleAttr.X));
				cargos.get(i).setDual(constraints[i].get(GRB.DoubleAttr.Pi));//
				cargos.get(i).setQuantityDual(quantityCon[i].get(GRB.DoubleAttr.Pi));
//				cargos.get(i).setDual2(constraints2[i].get(GRB.DoubleAttr.Pi));
//				System.out.println("Cargo dual "+i+": "+constraints[i].getDual());
//				System.out.println("Cargo quantity dual "+i+": "+quantityCon[i].getDual());
//				System.out.println("optional dual "+i+": "+constraints2[i].getDual());
				//constraints[i].print();
			}
			double supershipDual = 999999999;
//			for(int i = 0; i < ships.size(); i++) {
//				ships.get(0).setDual(convexity.getDual());
//				if(convexity.getDual()<supershipDual) {
//					supershipDual = convexity.getDual();
//				}
////				System.out.println("Ship dual "+i+": "+convexity[i].getDual());
//			}
		//	supership.setDual(supershipDual);
			Date date3 = new Date();
			tm.totalTimeInMaster += (date3.getTime()-date4.getTime());
			//System.out.println("time in master "+(date3.getTime()-date2.getTime()));
			//System.out.println("Time of master: "+tid2);
		}
		}
		//problem.print();
		//

		
		
		
//		if(problem.getObjVal()<0 && node.getDepth()>0 && counter2>1) {
//			node.setObjectiveValue(-1000);
//			node.setSolved(true);
//			return;
//		}
		XPRBbasis[] childbasis = {null, null};
//		XPRBbasis[] childbasis = {basis1, basis2};
		node.setChildBasis(childbasis);
		System.out.println("TIme in columnpool: "+this.timeUsed1);
		System.out.println("TIme in sub 1: "+this.timeUsed2);
		System.out.println("TIme in sub2: "+this.timeUsed3);
		int columnpoolsize  = 0;
//		for(int i = 0; i < ships.size(); i++) {
//			columnpoolsize+= ColumnPool.get(i).size();
//		}
		System.out.println("total size of column pool "+columnpoolsize);
		System.out.println("number of paths created in total "+pathList.size());
		if(counter2>=tm.MaxIter && counter>0) {
			node.setSolved(false);
		}
		else{
			node.setSolved(true);
		}
	//	checkForSubsetRowCuts();
		node.setObjectiveValue(model.get(DoubleAttr.ObjVal));
//		problem.print();
//		for(int i = 0; i < ships.size(); i++) {
//			file.writeTestOutput("ship dual "+i+": "+ships.get(i).getDual());
////			ships.get(i).setDual(0);
////			
//		}
//		for(int i = 0; i <cargos.size(); i++) {
//			//System.out.println("s_"+i+" got value "+optionalVars[i].get(GRB.DoubleAttr.X));
//			file.writeTestOutput("cargo dual "+i+": "+cargos.get(i).getDual());
//			file.writeTestOutput("cargo quantity dual "+i+": "+cargos.get(i).getQuantityDual());
////			cargos.get(i).setDual(0);
////			cargos.get(i).setQuantityDual(0);
//		}
//		file.writeTestOutput("COunter 2: "+counter2);
//		file.writeTestOutput("Is solved: "+node.isSolved());
//		file.writeTestOutput("NOde id: "+node.getNodeId());
//		file.writeTestOutput("Objective Value: "+node.getObjectiveValue());
//		file.writeTestOutput("number of cargoes upperbound "+node.getNumberOfCargoesBranchUpper());
//		file.writeTestOutput("number of cargoes lowerbound "+node.getNumberOfCargoesBranchLower());
//		for(int b = 0; b < node.getBranches().size(); b++) {
//			file.writeTestOutput("Branching on: "+tm.getBranches().get(node.getBranches().get(b)).getShip()+"   "+tm.getBranches().get(node.getBranches().get(b)).getCargo()+"  "+tm.getBranches().get(node.getBranches().get(b)).getDecision());
//		}
//		for(int b = 0; b < node.getCargoBranches().size(); b++) {
//			file.writeTestOutput("Branching on: "+tm.getCargoBranches().get(node.getCargoBranches().get(b)).getCargo()+"   "+tm.getCargoBranches().get(node.getCargoBranches().get(b)).getValue()+"  "+tm.getCargoBranches().get(node.getCargoBranches().get(b)).getDecision());
//		}
//		file.writeTestOutput("Node depth: "+node.getDepth());
//		
//		file.writeTestOutput("-------------------");
//		//file.writeTestOutput(problem.toString());
//		file.flush();
	}
	
//	
	

	
	private double[] branchingSelection(ArrayList<Integer> nonBinaryVariables, BBNode node) throws Exception{
		double[][] variablesum = new double[ships.size()][cargos.size()];
		
		found:
		for(int i : nonBinaryVariables) {
			Path p = pathList.get(allPaths.get(variables.get(i)));
			int ship = ships.indexOf(p.ship);
//			System.out.println("Ship index "+ship);
//			System.out.println)()
			if(ship>=0) {
				for(int c : p.cargosHandled) {
					variablesum[ship][c] += variables.get(i).get(GRB.DoubleAttr.X); //*Math.min(1, Math.ceil(p.ship.getOriginalVolumeCapacity()/cargos.get(c).getQuantity()));
				}
			}
		}
		int bestShip = 0;
		int bestCargo = 0;
		for(int i = 0; i < ships.size(); i++) {
			for(int j = 0; j< cargos.size(); j++) {
//				System.out.println(i+" "+j+" "+variablesum[i][j]);
				if(Math.abs(0.5-variablesum[i][j])<Math.abs(0.5-variablesum[bestShip][bestCargo])) {
//					for(int b = 0; b < node.getBranches().size(); b++) {
////						//System.out.println(tm.getBranches().get(node.getBranches().get(b)).getShip()+"   "+tm.getBranches().get(node.getBranches().get(b)).getCargo());
//						if(tm.getBranches().get(node.getBranches().get(b)).getShip().equalsIgnoreCase(ships.get(i).getName()) && j==tm.getBranches().get(node.getBranches().get(b)).getCargo()) {
////							//System.out.println("HERE WE ARE!!!!!!!!!!!!!!!!!!!!!!!");
//							break next;
//						}
//					}
					bestShip = i;
					bestCargo = j;
				}
			}
		}
		
		//if(variablesum[bestShip][bestCargo]<1-tm.getZeroTol() && variablesum[bestShip][bestCargo]>tm.getZeroTol()) {
			
			//System.out.println("Variable branch is "+bestShip+" "+bestCargo+" with sum "+ variablesum[bestShip][bestCargo]);
			
			double[] ple = {bestShip,bestCargo,variablesum[bestShip][bestCargo]};
			return ple;
//		}
//		else {
//			return null;
//		}
	}
	private double[][] getArcFlow(ArrayList<Integer> nonBinaryVariables) throws Exception{
        double[][] variablesum = new double[cargos.size()*2][cargos.size()*2+1];
		
		found:
		for(int i : nonBinaryVariables) {
			Path p = pathList.get(allPaths.get(variables.get(i)));
			//int ship = ships.indexOf(p.ship);
			int lastport = p.portList.get(1).getId();
			int currentport = -1;
			for(int j = 2; j < p.portList.size(); j++) {
				currentport = p.portList.get(j).getId();
				variablesum[lastport][currentport] += variables.get(i).get(GRB.DoubleAttr.X); //*Math.min(1, Math.ceil(p.ship.getOriginalVolumeCapacity()/cargos.get(c).getQuantity()));
				lastport = currentport;
			}
		
		}
	return variablesum; 
	}
	
	private double[] branchingSelectionArc(ArrayList<Integer> nonBinaryVariables, BBNode node) throws Exception{
		double[][] variablesum = new double[cargos.size()*2][cargos.size()*2+1];
		double[] firstsum = new double[cargos.size()];
		
		found:
		for(int i : nonBinaryVariables) {
			Path p = pathList.get(allPaths.get(variables.get(i)));
			int ship = ships.indexOf(p.ship);
			
			int lastport = p.portList.get(1).getId();
			firstsum[lastport] += variables.get(i).get(GRB.DoubleAttr.X);
			int currentport = -1;
			for(int j = 2; j < p.portList.size(); j++) {
				currentport = p.portList.get(j).getId();
				variablesum[lastport][currentport] += variables.get(i).get(GRB.DoubleAttr.X); //*Math.min(1, Math.ceil(p.ship.getOriginalVolumeCapacity()/cargos.get(c).getQuantity()));
				lastport = currentport;
			}
		
		}
		
		int besttail = 0;
		int besthead = 0;
		double mostFractional = 1.0;
		
//		for(int i = 0; i < cargos.size(); i++) {
//			if(Math.abs(0.5-(firstsum[i]-Math.floor(firstsum[i])))<mostFractional) {
//				besthead = i;
//				mostFractional = Math.abs(0.5-(firstsum[i]-Math.floor(firstsum[i])));
//				System.out.println("update most fractional "+mostFractional+" head: "+i);
//			}
//		}
		
//		for(int i = 0; i < ships.size(); i++) {
			for(int j = 0; j< cargos.size()*2; j++) {
				for(int k = 0; k< cargos.size()*2+1; k++) {
//				System.out.println(i+" "+j+" "+variablesum[i][j]);
				if(Math.abs(0.5-(variablesum[j][k]-Math.floor(variablesum[j][k])))<mostFractional) {
//					for(int b = 0; b < node.getBranches().size(); b++) {
////						//System.out.println(tm.getBranches().get(node.getBranches().get(b)).getShip()+"   "+tm.getBranches().get(node.getBranches().get(b)).getCargo());
//						if(tm.getBranches().get(node.getBranches().get(b)).getShip().equalsIgnoreCase(ships.get(i).getName()) && j==tm.getBranches().get(node.getBranches().get(b)).getCargo()) {
////							//System.out.println("HERE WE ARE!!!!!!!!!!!!!!!!!!!!!!!");
//							break next;
//						}
//					}
					
					besttail = j;
					besthead = k;
					mostFractional = Math.abs(0.5-(variablesum[j][k]-Math.floor(variablesum[j][k])));
				}
			}
//		}
		}
		//if(variablesum[bestShip][bestCargo]<1-tm.getZeroTol() && variablesum[bestShip][bestCargo]>tm.getZeroTol()) {
			
			System.out.println("Variable branch is "+besthead+" "+besttail+" with sum "+ mostFractional);
			System.out.println("first sum: "+Arrays.toString(firstsum));
			if(besttail==-1) {
				double[] ple = {besttail,besthead,firstsum[besthead]};
				return ple;
			}
			else {
			
				double[] ple = {besttail,besthead,variablesum[besttail][besthead]};
				return ple;
			}
//		}
//		else {
//			return null;
//		}
	}
	
	private double[] branchingSelectionArc2(ArrayList<Integer> nonBinaryVariables, BBNode node) throws Exception{
		double[][][] variablesum = new double[cargos.size()*2][cargos.size()*2][cargos.size()*2+1];
		
		found:
		for(int i : nonBinaryVariables) {
			Path p = pathList.get(allPaths.get(variables.get(i)));
			if(p.portList.size()>2) {
				int lastport = p.portList.get(1).getId();
				int currentport = p.portList.get(2).getId();;
				int nextport = -1;
				for(int j = 3; j < p.portList.size(); j++) {
					nextport = p.portList.get(j).getId();
					variablesum[lastport][currentport][nextport] += variables.get(i).get(GRB.DoubleAttr.X); //*Math.min(1, Math.ceil(p.ship.getOriginalVolumeCapacity()/cargos.get(c).getQuantity()));
					lastport = currentport;
					currentport = nextport;
				}
			}
		}
		
		int besttail = 0;
		int bestmiddle = 0;
		int besthead = 0;
		for(int i = 0; i < cargos.size()*2; i++) {
			for(int j = 0; j< cargos.size()*2; j++) {
				for(int k = 0; k< cargos.size()*2+1; k++) {
//				for(int k = 0; k< cargos.size()*2; k++) {
//				System.out.println(i+" "+j+" "+variablesum[i][j]);
				if(Math.abs(0.5-variablesum[i][j][k]-Math.floor(variablesum[i][j][k]))<Math.abs(0.5-variablesum[besttail][bestmiddle][besthead]-Math.floor(variablesum[besttail][bestmiddle][besthead]))) {
//					for(int b = 0; b < node.getBranches().size(); b++) {
////						//System.out.println(tm.getBranches().get(node.getBranches().get(b)).getShip()+"   "+tm.getBranches().get(node.getBranches().get(b)).getCargo());
//						if(tm.getBranches().get(node.getBranches().get(b)).getShip().equalsIgnoreCase(ships.get(i).getName()) && j==tm.getBranches().get(node.getBranches().get(b)).getCargo()) {
////							//System.out.println("HERE WE ARE!!!!!!!!!!!!!!!!!!!!!!!");
//							break next;
//						}
//					}
					
					besttail = i;
					bestmiddle = j;
					besthead = k;
				}
				}
		}
		}
		//if(variablesum[bestShip][bestCargo]<1-tm.getZeroTol() && variablesum[bestShip][bestCargo]>tm.getZeroTol()) {
			
			System.out.println("Two arc branch is "+besttail+" "+bestmiddle+" "+besthead+" with sum "+ variablesum[besttail][bestmiddle][besthead]);
			
			double[] ple = {besttail,bestmiddle, besthead,variablesum[besttail][bestmiddle][besthead]};
			return ple;
//		}
//		else {
//			return null;
//		}
	}
	
	private double[] branchingSelectionCargo(ArrayList<Integer> nonBinaryVariables, BBNode node) throws Exception{
		
		int bestOpt = 0; 
		//System.out.println("y "+0+ "  "+optionalVars[0].get(GRB.DoubleAttr.X));
		for(int i = 1; i<cargos.size(); i++) {
			//System.out.println("y "+i+ "  "+optionalVars[i].get(GRB.DoubleAttr.X));
			if(Math.abs(optionalVars[i].get(GRB.DoubleAttr.X)-0.5)<Math.abs(optionalVars[bestOpt].get(GRB.DoubleAttr.X)-0.5)) {
				bestOpt = i;
			}
		}
		if(optionalVars[bestOpt].get(GRB.DoubleAttr.X)<1-tm.getZeroTol() && optionalVars[bestOpt].get(GRB.DoubleAttr.X)>tm.getZeroTol()) {
			double[] temp = {bestOpt,optionalVars[bestOpt].get(GRB.DoubleAttr.X)};
			//System.out.println("Variable Cargo branch is "+bestOpt+" with sum "+ optionalVars[bestOpt].get(GRB.DoubleAttr.X));
			return temp;
		}
//		else {
//			double[] temp = {0.0,0.0};
//			return temp;
//		}
		
//		double[] variablesum = new double[cargos.size()];
//		found:
//		for(int i : nonBinaryVariables) {
//			Path p = pathList.get(allPaths.get(variables.get(i)));
//			for(int c = 0; c < cargos.size(); c++) {
//				variablesum[c] += p.getCargoesHandled()[c]*variables.get(i).get(GRB.DoubleAttr.X); //*Math.min(2,Math.ceil(p.ship.getOriginalVolumeCapacity()/cargos.get(c).getQuantity()));
//			}
//		
//		}
		
		int bestCargo = 0;
		//System.out.println("cargo "+0+"   "+variablesum[0]);
			for(int j = 1; j< cargos.size(); j++) {
				System.out.println("cargo "+j+"   "+z[j].get(GRB.DoubleAttr.X)); //+ "    "+Math.abs(variablesum[j] - Math.floor(variablesum[j]-0.5)));
				if(Math.abs(z[j].get(GRB.DoubleAttr.X) - Math.floor(z[j].get(GRB.DoubleAttr.X))-0.5) < Math.abs(z[bestCargo].get(GRB.DoubleAttr.X) - Math.floor(z[bestCargo].get(GRB.DoubleAttr.X))-0.5)) {
//					for(int b = 0; b < node.getBranches().size(); b++) {
////						//System.out.println(tm.getBranches().get(node.getBranches().get(b)).getShip()+"   "+tm.getBranches().get(node.getBranches().get(b)).getCargo());
//						if(tm.getBranches().get(node.getBranches().get(b)).getShip().equalsIgnoreCase(ships.get(i).getName()) && j==tm.getBranches().get(node.getBranches().get(b)).getCargo()) {
////							//System.out.println("HERE WE ARE!!!!!!!!!!!!!!!!!!!!!!!");
//							break next;
//						}
//					}
					
					bestCargo = j;
				}
			}
		//System.out.println("Variable Cargo branch is "+bestCargo+" with sum "+ variablesum[bestCargo]);
//		if((variablesum[bestCargo] - Math.floor(variablesum[bestCargo]))>0.01
//				&& Math.ceil(variablesum[bestCargo])-variablesum[bestCargo]>0.01) {
			System.out.println("Variable Cargo branch is "+bestCargo+" with sum "+ z[bestCargo].get(GRB.DoubleAttr.X));
			double[] ple = {bestCargo, z[bestCargo].get(GRB.DoubleAttr.X)};
			return ple;
//		}
//		else {
//			return null;
//		} */
	}	
private double branchingSelectionNumberOfCargoes() throws Exception{
		
		double sum = 0; 
		for(int i = 0; i<cargos.size(); i++) {
			sum+=optionalVars[i].get(GRB.DoubleAttr.X);
			//sum+=z[i].get(GRB.DoubleAttr.X);
		}
//		System.out.println("sum of cargoes is "+sum);
//		if((sum - Math.floor(sum))>0.01
//				&& Math.ceil(sum)-sum>0.01) {
			//System.out.println("Branching on Number of Cargoes");
			
			return sum;
//		}
//		else {
//			return -1;
//		}
	}

	
	private BBNode[] branchOnArc(BBNode node, int[] branch) {
		BBNode nodeRight = new BBNode(node, node.getDepth()+1, tm.nodecount+1, node.getNumberOfCargoesBranchUpper(), node.getNumberOfCargoesBranchLower());
		tm.updateNodeCount();
		BBNode nodeLeft = new BBNode(node, node.getDepth()+1, tm.nodecount+1, node.getNumberOfCargoesBranchUpper(), node.getNumberOfCargoesBranchLower());
		tm.updateNodeCount();
		copyNode(node, nodeLeft, nodeRight);
		
		
		int branchleft = -1;
		int branchright = -1;
		
		branchleft = tm.addBranch(new Arcbranch(branch[0],  1, branch[1], branch[2]+1));
				
		branchright = tm.addBranch(new Arcbranch(branch[0],  -1, branch[1], branch[2]));
//		
		nodeLeft.getBranches().add(branchleft);
		nodeRight.getBranches().add(branchright);
	
		BBNode[] temp = {nodeLeft, nodeRight};
		return temp;
	}
	
	private BBNode[] branchOnTwoArc(BBNode node, int[] branch) {
		BBNode nodeRight = new BBNode(node, node.getDepth()+1, tm.nodecount+1, node.getNumberOfCargoesBranchUpper(), node.getNumberOfCargoesBranchLower());
		tm.updateNodeCount();
		BBNode nodeLeft = new BBNode(node, node.getDepth()+1, tm.nodecount+1, node.getNumberOfCargoesBranchUpper(), node.getNumberOfCargoesBranchLower());
		tm.updateNodeCount();
		copyNode(node, nodeLeft, nodeRight);
		
		
		int branchleft = -1;
		int branchright = -1;
		
		branchleft = tm.addBranch(new TwoArcBranch(branch[0],  1, branch[1], branch[2], branch[3]+1));
				
		branchright = tm.addBranch(new TwoArcBranch(branch[0],  -1, branch[1], branch[2], branch[3]));
//		
		nodeLeft.getBranches().add(branchleft);
		nodeRight.getBranches().add(branchright);
	
		BBNode[] temp = {nodeLeft, nodeRight};
		return temp;
	}
	
	
	
	private BBNode[] branchOnCargo(BBNode node, int[] branch) {
		BBNode nodeRight = new BBNode(node, node.getDepth()+1, tm.nodecount+1, node.getNumberOfCargoesBranchUpper(), node.getNumberOfCargoesBranchLower());
		tm.updateNodeCount();
		BBNode nodeLeft = new BBNode(node, node.getDepth()+1, tm.nodecount+1, node.getNumberOfCargoesBranchUpper(), node.getNumberOfCargoesBranchLower());
		tm.updateNodeCount();
		copyNode(node, nodeLeft, nodeRight);
		
		int branchleft = -1;
		int branchright = -1;
		
		//if(nonBinaryVariables.size()>0) {
			//int[] branch = branchingSelectionCargo(nonBinaryVariables, node);
			if(branch==null) {  //|| branch[1]>=2)  {
				return null;
			}
			else {
				//System.out.println("Should not be here");
//				if(branch[1]<2) {
//					branchleft = tm.addCargoBranch(new CargoBranch(branch[0], 1, 2));
//					branchright = tm.addCargoBranch(new CargoBranch(branch[0],-1,0));
//				}
//				else {
					branchleft = tm.addCargoBranch(new CargoBranch(branch[0], 1, branch[1]+1));
					branchright = tm.addCargoBranch(new CargoBranch(branch[0],-1,branch[1]));
//				}
		}
		//}
		nodeLeft.getCargoBranches().add(branchleft);
		nodeRight.getCargoBranches().add(branchright);
		
		//lastnode = node;
		if(branchleft<0) {
			//node.setSolution(solution);
			//node.setSolutionvars(solutionvars);
			if(!node.isSolved()) {
				System.out.println("Waaatch out here!!!!!!!!!!");
			}
			return null;
		}
		
//		Vector<Integer> left = new Vector<Integer>();
//		Vector<Integer> right = new Vector<Integer>();
		
		
	
		BBNode[] temp = {nodeLeft, nodeRight};
		return temp;
	}
	
	private BBNode[] branchOnNumVehicles(BBNode node, int branch) {
		BBNode nodeRight = new BBNode(node, node.getDepth()+1, tm.nodecount+1, node.getNumberOfCargoesBranchUpper(), node.getNumberOfCargoesBranchLower());
		tm.updateNodeCount();
		BBNode nodeLeft = new BBNode(node, node.getDepth()+1, tm.nodecount+1, node.getNumberOfCargoesBranchUpper(), node.getNumberOfCargoesBranchLower());
		tm.updateNodeCount();
		copyNode(node, nodeLeft, nodeRight);

		
		nodeLeft.minVehicles = branch+1;
		nodeRight.maxVehicles = branch;
		
		BBNode[] temp = {nodeLeft, nodeRight};
		return temp;
	}
	
//	private BBNode[] branchOnNumberOfCargo(BBNode node, int branch) {
//		BBNode nodeRight = new BBNode(node, node.getDepth()+1, tm.nodecount+1, node.getNumberOfCargoesBranchUpper(), node.getNumberOfCargoesBranchLower());
//		nodeRight.setBasis(node.getChildBasis()[0]);
//		nodeRight.setGeneratedVariables(new Vector<Integer>());
//		tm.updateNodeCount();
//		BBNode nodeLeft = new BBNode(node, node.getDepth()+1, tm.nodecount+1, node.getNumberOfCargoesBranchUpper(), node.getNumberOfCargoesBranchLower());
//		nodeLeft.setGeneratedVariables(new Vector<Integer>());
//		nodeLeft.setBasis(node.getChildBasis()[1]);
//		tm.updateNodeCount();
//		
//		int branchleft = -1;
//		int branchright = -1;
//		
//		
//			//int branch = branchingSelectionNumberOfCargoes();
//			if(branch<0) {  //|| branch[1]>=2)  {
//				return null;
//			}
//			
//		Vector<Integer> branchesleft = new Vector<Integer>(node.getBranches());
//		Vector<Integer> cargobranchesleft = new Vector<Integer>(node.getCargoBranches());
//		
//		
//		Vector<Integer> cargobranchesright = new Vector<Integer>(node.getCargoBranches());
//		Vector<Integer> branchesright = new Vector<Integer>(node.getBranches());
//		
//		
//		nodeLeft.setBranches(branchesleft);
//		nodeLeft.setCargoBranches(cargobranchesleft);
//		nodeRight.setBranches(branchesright);
//		nodeRight.setCargoBranches(cargobranchesright);
//		
//		nodeLeft.setNumberOfCargoesBranchUpper(branch);
//		nodeRight.setNumberOfCargoesBranchLower(branch+1);
//		
//		Vector<Integer> left = new Vector<Integer>();
//		Vector<Integer> right = new Vector<Integer>();
//		
//	
//		BBNode[] temp = {nodeLeft, nodeRight};
//		return temp;
//	}
	
//	private GRBVar searchColumnPool(Vector<Integer> v, BBNode node) {
//		GRBVar best = null;
//		double bestvalue = -1000;
//		for(int i : v) {
//			GRBVar tempvar = variables.get(i);
//			double temp = route.pathPrice(pathList.get(allPaths.get(tempvar)), node, tm);
//			if(temp>bestvalue) {
//				bestvalue = temp;
//				best = tempvar;
//			}
//		}
//		if(bestvalue>0) {
//			return best;
//		}
//		else {
//			return null;
//		}
//	}
	private void removeVariablesFromProblem(int numberOfColumns) throws Exception{
		PriorityQueue<GRBVar> priQueue = new PriorityQueue<GRBVar>(variables.size(), new ReducedCostComparator());
		if(variables.size()<numberOfColumns) {
			return;
		}
		for(GRBVar var : variables) {
			priQueue.add(var);
		}
		int counter = 0;
		while(!priQueue.isEmpty()) {
			GRBVar var = priQueue.poll();
			if(counter>numberOfColumns) {
				var.set(GRB.DoubleAttr.UB,0);
			}
			counter++;
		}
	}
//	private ArrayList<Path> mainColumns(Vector<Integer> paths, BBNode node) {
//		ArrayList<Path> availablePaths = new ArrayList<Path>(); 
//		Vector<Integer> tempVector = new Vector<Integer>();
//		for(int i : paths) {
//			Path p = pathList.get(i);
//			double temp = route.pathPrice(p, node, tm);
//			
//			if(temp>tm.getZeroTol()) {
//				availablePaths.add(p);
//				tempVector.add(i);
//			}
//			else if(temp==-XPRB.INFINITY){
//				tempVector.add(i);
//				node.getBranchVariables().add(i);
//			}
//		}
//		paths.removeAll(tempVector);
//		return availablePaths;
//	}
	private void preProcessNode(BBNode node) {
		
	}
	
	private void initiateProblem() throws Exception{
		route.setArcdualvalues(new double[cargos.size()*2][cargos.size()*2+1], new double[cargos.size()], new double[cargos.size()*2][cargos.size()*2][cargos.size()*2+1]);
		route.setShipCargoDualvalues(new double[ships.size()][cargos.size()]);
		route2.setArcdualvalues(new double[cargos.size()*2][cargos.size()*2+1], new double[cargos.size()],new double[cargos.size()*2][cargos.size()*2][cargos.size()*2+1]);
		route2.setShipCargoDualvalues(new double[ships.size()][cargos.size()]);
		route3.setArcdualvalues(new double[cargos.size()*2][cargos.size()*2+1], new double[cargos.size()],new double[cargos.size()*2][cargos.size()*2][cargos.size()*2+1]);
		route3.setShipCargoDualvalues(new double[ships.size()][cargos.size()]);
//		this.problem = xprb.newProb("problem 1");
//		problem.setCutMode(1);
//		this.problem.setXPRSintControl(XOctrl.XPRS_PRESOLVE, 1);
//		this.problem.setXPRSintControl(XOctrl.XPRS_OUTPUTLOG, 0);
//		this.problem.setXPRSdblControl(XPRS.OPTIMALITYTOL, 0.00000000001);
////		this.problem.setDictionarySize(XPRB.DICT_NAMES, 0);
//		this.problem.setColOrder(1);
		this.obj = new GRBLinExpr();
//		this.quantity = new GRBLinExpr[cargos.size()];
//		this.expr2 = new GRBLinExpr[cargos.size()];
//		this.expr = new GRBLinExpr[cargos.size()];
//		this.Optionalexpr = new GRBLinExpr[cargos.size()];
//		this.exprcon = new GRBLinExpr[ships.size()];
//		this.capacity = new GRBLinExpr[ships.size()];
//		this.numberOfCargoesExpr = new GRBLinExpr();
		this.variables = new ArrayList<GRBVar>();
//		this.quantityVars = new ArrayList<GRBVar>();
		this.optionalVars = new GRBVar[cargos.size()];
		this.z = new GRBVar[cargos.size()];
		this.constraints = new GRBConstr[cargos.size()];
//		this.constraints2 = new GRBConstr[cargos.size()];
		this.optionalcon= new GRBConstr[cargos.size()];
//		this.convexity = problem.newCtr("convexity");
		this.quantityCon = new GRBConstr[cargos.size()];
		this.capacityCon = new GRBConstr[ships.size()];
		
		this.quantitySlack = new GRBVar[cargos.size()];
		this.branchingslack = new GRBVar[2][cargos.size()*2][cargos.size()*2+1];
		this.twobranchingslack = new GRBVar[2][cargos.size()*2][cargos.size()*2][cargos.size()*2+1];
		this.slack = new GRBVar[cargos.size()];
		this.slack2 = new GRBVar[cargos.size()];
//		this.numberOfCargoesCon = problem.newCtr("Number of Cargoes con ");
//		this.numberOfCargoesCon.setType(XPRB.E);
//		this.numberOfCargoesCon.add(0);
//		this.numberOfCargoes = this.problem.newVar("number",XPRB.PL);
//		numberOfCargoesCon.addTerm(numberOfCargoes,-1);
//		
//		this.arcBranches = new GRBConstr[cargos.size()][cargos.size()];
		
		
//		int[] mincargocoeff = {1,2,1,1,1,1,1,1,1,1,1,1};
		this.conSlack = model.addVar(0,GRB.INFINITY,0,GRB.CONTINUOUS,"Con slack");
		this.numVehicles = model.addVar(0,GRB.INFINITY,0,GRB.CONTINUOUS,"Num vehicles");
		model.update();
		GRBLinExpr ple = new GRBLinExpr();
//		ple.addTerm(-1,this.numVehicles);
		System.out.println(this.numVehicles.toString());
		this.convexity = model.addConstr(ple,GRB.EQUAL,0,"Convexity");
		model.update();
		model.chgCoeff(this.convexity, this.numVehicles,-1);
		model.chgCoeff(this.convexity, this.conSlack,1);
		this.obj.addTerm(-999999, this.conSlack);
		for(int i = 0; i < cargos.size(); i++) {
			constraints[i] = model.addConstr(new GRBLinExpr(), GRB.EQUAL,0,"con"+i);
//			constraints2[i] = model.addConstr(new GRBLinExpr(), GRB.GREATER_EQUAL,0,"con2"+i);
//			constraints2[i] = problem.newCtr("con2"+i);
//			constraints2[i].setType(XPRB.N);
//			constraints2[i].add(0);
			//constraints[i].assign(expr[i].gEql(0));
			quantityCon[i] = model.addConstr(new GRBLinExpr(), GRB.GREATER_EQUAL,0,"quantity con"+i);
//			quantityCon[i].setType(XPRB.G);
//			quantityCon[i].add(0);
//			GRBVar slack = problem.newVar("slack "+i,XPRB.PL,0, 10000);
//			quantity[i].addTerm(slack, 1);
//			obj.addTerm(slack, -10000);
			//quantityCon[i].assign(quantity[i].eql(0));
			//constraints[i].assign(expr[i].lEql(1));
		}
		model.update();
		for(int i = 0; i < cargos.size(); i++) {
//			this.expr[i] = new GRBLinExpr();
//			this.expr2[i] = new GRBLinExpr();
//			this.Optionalexpr[i] = new GRBLinExpr();
			this.optionalVars[i] = model.addVar(0,1,0,GRB.CONTINUOUS,"s_"+i);
			model.update();
			this.optionalcon[i] = model.addConstr(this.optionalVars[i],GRB.LESS_EQUAL,0,"optional constraint "+i);
			model.update();
//			this.optionalcon[i].setType(XPRB.L);
//			this.optionalcon[i].add(0);
			
//			if(cargos.get(i).getOptional()==0) {
//				this.optionalVars[i].fix(1.0);
//			}
			this.z[i] = model.addVar(0,GRB.INFINITY,0,GRB.CONTINUOUS, "z_"+i);
			model.update();
//			this.z[i] = this.problem.newVar("z_"+i, XPRB.PL);
			this.obj.addTerm(cargos.get(i).getIncome(), this.optionalVars[i]);
			//this.expr[i].addTerm(this.optionalVars[i], -1);
			model.chgCoeff(this.optionalcon[i],z[i], -1);
			if(cargos.get(i).getQuantity()>ships.get(0).getOriginalVolumeCapacity()+tm.getZeroTol()) {
				model.chgCoeff(this.optionalcon[i],optionalVars[i], 2);
			}
			else {
				model.chgCoeff(this.optionalcon[i],optionalVars[i], 1);
			}
			//this.Optionalexpr[i].addTerm(optionalVars[i], mincargocoeff[i]);
//			quantityCut.addTerm(optionalVars[i], quantityCutCoeff[i]);
//			quantityCut2.addTerm(optionalVars[i], quantityCutCoeff2[i]);
//			quantityCut3.addTerm(optionalVars[i], quantityCutCoeff3[i]);
//			quantityCut4.addTerm(optionalVars[i], quantityCutCoeff4[i]);
			
			model.chgCoeff(this.constraints[i], this.z[i], -1);
//			model.chgCoeff(this.constraints2[i], this.optionalVars[i], -1);
//			this.constraints[i].addTerm(this.z[i], -1);
//			this.constraints2[i].addTerm(this.optionalVars[i], -1);
//			this.quantityCon[i] = new GRBLinExpr();
			model.chgCoeff(this.quantityCon[i],this.optionalVars[i], -1*cargos.get(i).getQuantity());
//			this.numberOfCargoesCon.addTerm(optionalVars[i],1);
			//this.numberOfCargoesExpr.addTerm(z[i],1);
			this.quantitySlack[i] = model.addVar(0,GRB.INFINITY,0,GRB.CONTINUOUS, "qs_"+i);
			this.slack[i] = model.addVar(0,GRB.INFINITY,0,GRB.CONTINUOUS, "s_"+i);
			this.slack2[i] = model.addVar(0,GRB.INFINITY,0,GRB.CONTINUOUS, "s2_"+i);
			model.update();
			this.obj.addTerm(-999999, this.slack[i]);
			this.obj.addTerm(-999999, this.slack2[i]);
			this.obj.addTerm(-999999, this.quantitySlack[i]);
			model.chgCoeff(this.constraints[i],this.slack[i], 1);
			model.chgCoeff(this.constraints[i],this.slack2[i], -1);
//			model.chgCoeff(this.constraints2[i],this.slack2[i], 1);
			model.chgCoeff(this.quantityCon[i],this.quantitySlack[i], 1);
		}
//		quantityCutCon.assign(quantityCut.lEql(4));
//		quantityCutCon2.assign(quantityCut2.lEql(4));
//		quantityCutCon3.assign(quantityCut.lEql(4));
//		quantityCutCon4.assign(quantityCut2.lEql(3));
		for(int s = 0; s < ships.size(); s++) {
//			ColumnPool.add(new ArrayList<Path>());
//			exprcon[s] = new GRBLinExpr();
//			ships.get(s).setOriginalVolumeCapacity(ships.get(s).getVolumeCapacity());
		}
//		for(int i = 0; i < ships.size(); i++) {
//			convexity[i] = problem.newCtr("convexity con");
//			convexity[i].setType(XPRB.L);
//			convexity[i].add(1);
			//convexity[i].assign(exprcon[i].lEql(1));
//		}
		
		Preprocess pre = new Preprocess(ports, cargos, ships);
//		cluster1 = pre.make2cluster();
//		System.exit(0);
//		if(tm.useVolumeCuts) {
//			ArrayList<Vector<Integer>> volumeCuts = pre.getTotalVolumeCuts();
//			tm.volumeCuts = volumeCuts.size();
//			GRBConstr[] volumeCons = new GRBConstr[volumeCuts.size()];
//			for(int i = 0; i < volumeCuts.size(); i++) {
//				volumeCons[i]= problem.newCtr("Volumecut "+i);
//				volumeCons[i].setType(XPRB.L);
//				volumeCons[i].add(volumeCuts.get(i).size()-1);
//				volumeCons[i].setModCut(true);
//				for(int v : volumeCuts.get(i)) {
//					volumeCons[i].addTerm(optionalVars[v],1);
//				}
//			}
//			for(int i = 0; i < volumeCuts.size(); i++) {
//				volumeCons[i].print();
//			}
//			System.out.println("Number of volume cuts: "+volumeCuts.size());
//		}
//		System.exit(0);
//		initiateTwoPathCuts();
//		initiateCapacityCuts();
//		addShipCargoCuts();
//		addNonOverFullGroupCuts();
//		addNonOverFullGroupCuts2();
		
	}
	
	private void compareTwoPaths(ArrayList<Path> p, ArrayList<Path> temppath) {
		boolean found = true;
//		if(p.ship.getName().equalsIgnoreCase(temppath.ship.getName()) && p.portList.size() == temppath.portList.size()) {
//			for(int i = 1; i < p.portList.size(); i++) {
//				if(p.getPortList().get(i).getId()!= temppath.portList.get(i).getId() || 
//						(p.getPortList().get(i).getId()<cargos.size() && p.cargoVolume[p.getPortList().get(i).getId()]!= temppath.cargoVolume[temppath.getPortList().get(i).getId()])) {
//					found = false;
//					break;
//				}
//			}
		if(p.isEmpty() && !temppath.isEmpty()) {
			System.out.println("Path is null");
			System.out.println(temppath.toString());
			System.exit(0);
			found = false;
			
		}
		else if(!p.isEmpty() && temppath.isEmpty()) {
			System.out.println("Tempath is null");
			System.out.println(p.toString());
			System.exit(0);
			
		}
		else if(p.isEmpty() && temppath.isEmpty()) {
			return;
		}
		
		if(p.get(0).getDualLowerBound()-temppath.get(0).getDualLowerBound()>tm.getZeroTol() || p.get(0).getDualLowerBound()-temppath.get(0).getDualLowerBound()<-tm.getZeroTol() ) {
//			if(!found) {
				System.out.println("These columns are different!!!!");
				System.out.println(p.toString());
				System.out.println(temppath.toString());
//				System.out.println("Reduced cost of var: "+var.getRCost());
//				System.out.println("Value of var: "+var.get(GRB.DoubleAttr.X));
//				System.out.println("upperbound on var "+var.getUB());
				System.out.println("ship capacity "+p.get(0).ship.getOriginalVolumeCapacity());
				System.out.println("Ship dual: "+p.get(0).ship.getDual());
				for(int i = 0; i < cargos.size();i++) {
					System.out.println("Cargo size: "+cargos.get(i).getQuantity());
					System.out.println("Cargo dual "+i+": "+cargos.get(i).getDual());
					System.out.println("Cargo dual2 "+i+": "+cargos.get(i).getDual2());
					System.out.println("Cargo quantity dual "+i+": "+cargos.get(i).getQuantityDual());
			}
//				System.out.println(p.get(0).ship.getOriginalcostMatrix()[10][40]);
//				System.out.println(p.get(0).ship.getCostMatrix()[10][40]);
//				System.out.println(p.get(0).ship.getOriginalcostMatrix()[40][10]);
//				System.out.println(p.get(0).ship.getCostMatrix()[40][10]);
//				System.out.println(p.get(0).ship.getTimeInPort()[10]);
//				System.out.println(p.get(0).ship.getTimeInPort()[40]);
//				System.out.println(temppath.get(0).ship.getOriginalcostMatrix()[10][40]);
//				System.out.println(temppath.get(0).ship.getCostMatrix()[10][40]);
//				System.out.println(temppath.get(0).ship.getOriginalcostMatrix()[40][10]);
//				System.out.println(temppath.get(0).ship.getCostMatrix()[40][10]);
//				System.out.println(temppath.get(0).ship.getTimeInPort()[10]);
//				System.out.println(temppath.get(0).ship.getTimeInPort()[40]);
//				
				route.recalcPath(temppath.get(0));
				
				System.exit(0);
			}
		System.out.println(p.get(0).toString());
		System.out.println(temppath.get(0).toString());
	}
	
	
	private void comparePaths(Path p, BBNode node) throws Exception{
		for(GRBVar var : variables) {
			Path temppath = pathList.get(allPaths.get(var));
			boolean found = true;
			if(p.ship.getName().equalsIgnoreCase(temppath.ship.getName()) && p.portList.size() == temppath.portList.size()) {
				for(int i = 1; i < p.portList.size(); i++) {
					if(p.getPortList().get(i).getId()!= temppath.portList.get(i).getId() || 
							(p.getPortList().get(i).getId()<cargos.size() && p.cargoVolume[p.getPortList().get(i).getId()]!= temppath.cargoVolume[temppath.getPortList().get(i).getId()])) {
						found = false;
						break;
					}
				}
				if(found) {
					System.out.println("This column already exists!!!!");
					System.out.println(p.toString());
					System.out.println(temppath.toString());
//					System.out.println(arcBranches[1][4][28].get(GRB.StringAttr.values()));
					System.out.println("Reduced cost of var: "+var.get(GRB.DoubleAttr.RC));
					System.out.println("Value of var: "+var.get(GRB.DoubleAttr.X));
//					System.out.println("upperbound on var "+var.get(GRB.DoubleAttr.UB));
////					System.out.println("ship capacity "+p.ship.getOriginalVolumeCapacity());
					System.out.println("Ship dual: "+p.ship.getDual());
					for(int i = 0; i < cargos.size();i++) {
						System.out.println("Cargo size: "+cargos.get(i).getQuantity());
						System.out.println("Cargo dual "+i+": "+cargos.get(i).getDual());
						System.out.println("Cargo dual2 "+i+": "+cargos.get(i).getDual2());
						System.out.println("Cargo quantity dual "+i+": "+cargos.get(i).getQuantityDual());
					}
//					int[] fullCargo = {0,0,0,0,0,0,0,0,0,0,0,1};
					//route.knp.solveKnapsackProblem(temppath, null, fullCargo);
					if(nonOverFullGroupCuts3!=null) {
						for(int i = 0; i < cargos.size();i++) {
							if(nonOverFullGroupCuts3[i]!=null) {
								System.out.println("cut3 "+i+"  "+nonOverFullGroupCuts3[i].get(DoubleAttr.Pi));
						
							}
						}
					}
					for(int i = 0; i < cargos.size()*2; i++) {
						for(int j = 0; j < cargos.size()*2+1; j++) {
							if(arcBranches[0][i][j]!=null && arcBranches[0][i][j].get(GRB.DoubleAttr.Pi)>tm.getZeroTol()) {
								System.out.println("arc dual "+i+" "+j+": "+arcBranches[0][i][j].get(GRB.DoubleAttr.Pi));
							}
							if(arcBranches[1][i][j]!=null && arcBranches[1][i][j].get(GRB.DoubleAttr.Pi)<-tm.getZeroTol()) {
								System.out.println("arc dual "+i+" "+j+": "+arcBranches[1][i][j].get(GRB.DoubleAttr.Pi));
							}
						}
					}
					
					for(int i = 0; i < cargos.size()*2; i++) {
						for(int j = 0; j < cargos.size()*2; j++) {
							for(int k = 0; k < cargos.size()*2+1; k++) {
							if(twoArcBranches[0][i][j][k]!=null && twoArcBranches[0][i][j][k].get(GRB.DoubleAttr.Pi)>tm.getZeroTol()) {
								System.out.println("two arc dual "+i+" "+j+" "+k+": "+twoArcBranches[0][i][j][k].get(GRB.DoubleAttr.Pi));
							}
							if(twoArcBranches[1][i][j][k]!=null && twoArcBranches[1][i][j][k].get(GRB.DoubleAttr.Pi)<-tm.getZeroTol()) {
								System.out.println("two arc dual "+i+" "+j+" "+k+": "+twoArcBranches[1][i][j][k].get(GRB.DoubleAttr.Pi));
							}
							}
						}
					}
//					for(int i = 0; i < cargos.size();i++) {
//						System.out.println("nonoverfullcargocuts2 dual "+i+": "+nonOverFullGroupCuts2[i].getDual());
//					}
					for(Vector<Integer> vec : two.subsets) {
						if(cutConstraints.get(vec).get(GRB.DoubleAttr.Pi)<-tm.getZeroTol()) {
							System.out.println(vec.toString() +" "+cutConstraints.get(vec).get(GRB.DoubleAttr.Pi));
						}
					}
//					for(int s = 0; s < ships.size(); s++) {
//						for(int i = 0; i < cargos.size();i++) {
//							if(nonOverFullGroupCuts[s][i]!=null) {
//								if(nonOverFullGroupCuts[s][i].getDual()!=0.0) {
//									System.out.println(s+" "+i+":  "+nonOverFullGroupCuts[s][i].getDual());
//								}
//							}
//							for(int j = 0; j < cargos.size();j++) {
//								if(shipCargoCuts[s][i][j]!= null) {
//									if(shipCargoCuts[s][i][j].getDual()!= 0.0) {
//									System.out.println(s+" "+i+" "+j+":  "+shipCargoCuts[s][i][j].getDual());
//									}
//							
//								}
//							}
//						}
//					}
//					System.out.println("Subset Row duals");
//					for( int i = 0; i < cargos.size(); i++) {
//						for( int j = i+1; j < cargos.size(); j++) {
//							for( int k = j+1; k < cargos.size(); k++) {
//								if(subsetRowCuts[i][j][k]!=null) {
//									if(subsetRowCuts[i][j][k].get(GRB.DoubleAttr.Pi)>tm.getZeroTol()) {
//										System.out.println(i+" "+j+" "+k+": "+subsetRowCuts[i][j][k].get(GRB.DoubleAttr.Pi));
//									}
//								}
//							}
//						}
//					}
//					for(int i = 0; i < 3; i++) {
//						System.out.println("Check con "+i+" dual "+checkCon[i].getDual());
//					}
//					double routePathPrice = route.pathPrice(p, node, tm);
//					System.out.println("is not overfull "+route.checkForNonOverFullGroup(1, p));
//					System.out.println("Recalculated cost: "+routePathPrice);
////					System.out.println("checkcon dual "+checkCon.getDual());
////					checkCon.print();
////					problem.print();
//					file.writeTestOutput("node stop: ");
//					file.writeTestOutput("Is solved: "+node.isSolved());
//					file.writeTestOutput("NOde id: "+node.getNodeId());
//					file.writeTestOutput("Objective Value: "+node.getObjectiveValue());
//					for(int c = 0; c < node.getBranches().size(); c++) {
//						file.writeTestOutput("Branching on: "+tm.getBranches().get(node.getBranches().get(c)).getShip()+"   "+tm.getBranches().get(node.getBranches().get(c)).getCargo()+"  "+tm.getBranches().get(node.getBranches().get(c)).getDecision());
//					}
//					for(int c = 0; c < node.getCargoBranches().size(); c++) {
//						file.writeTestOutput("Branching on: "+tm.getCargoBranches().get(node.getCargoBranches().get(c)).getCargo()+"   "+tm.getCargoBranches().get(node.getCargoBranches().get(c)).getValue()+"  "+tm.getCargoBranches().get(node.getCargoBranches().get(c)).getDecision());
//					}
//					file.writeTestOutput("Node depth: "+node.getDepth());
//					
//					file.writeTestOutput("-------------------");
//					file.flush();
//					if(p.getDualLowerBound()-routePathPrice>tm.getZeroTol() || p.getDualLowerBound()-routePathPrice<-tm.getZeroTol()) {
//						System.out.println(var.getName());
//						System.exit(0);
//					}
					System.exit(0);
					return;
				}
			}
		}
	}

	private void comparePaths2(Path p, BBNode node) throws Exception{
		boolean test = false;
		for(GRBVar var : variables) {
			Path temppath = pathList.get(allPaths.get(var));
			boolean found = true;
			if(p.ship.getName().equalsIgnoreCase(temppath.ship.getName()) && p.portList.size() == temppath.portList.size()) {
				for(int i = 1; i < p.portList.size(); i++) {
					if(p.getPortList().get(i).getId()!= temppath.portList.get(i).getId()  || 
							(p.getPortList().get(i).getId()<cargos.size() && p.cargoVolume[p.getPortList().get(i).getId()]!= temppath.cargoVolume[temppath.getPortList().get(i).getId()])) {
						found = false;
						break;
					}
				}
				if(found) {
					System.out.println("found path");
					System.out.println(temppath.toString());
					test = true; 
					break;
					
//					System.out.println(temppath.toString());
//					
				}
			}
		}
		if(!test) {
			System.out.println("This column does not exist!!!!");
			System.out.println(p.toString());
		}
	}
//	private int addQuantityCuts(ArrayList<GRBVar> vars, Hashtable<GRBVar,GRBVar[]> qVars) {
//		int counter = 0;
//		for(GRBVar var: vars) {
//			GRBVar[] qvar = qVars.get(var);
//			for(int i = 0; i<cargos.size(); i++) {
//				if(qvar[i]!=null) {
//					if(qvar[i].get(GRB.DoubleAttr.X)-tm.getZeroTol()> var.get(GRB.DoubleAttr.X)*Math.min(pathList.get(allPaths.get(var)).ship.getOriginalVolumeCapacity(),cargos.get(i).getQuantity())) {
//				
//						GRBLinExpr tempexpr = new GRBLinExpr();
//						tempexpr.addTerm(1, qvar[i]);
//						tempexpr.addTerm(-1*Math.min(pathList.get(allPaths.get(var)).ship.getOriginalVolumeCapacity(),cargos.get(i).getQuantity()),var);
//						GRBConstr tempcon = model.addConstr(temp, sense, expr, name);
//						tempcon.assign(tempexpr.lEql(0));
//						counter ++;
////						System.out.println("Creating cut for "+var.getName()+" "+i);
////						System.out.println(qvar[i].get(GRB.DoubleAttr.X));
////						System.out.println(var.get(GRB.DoubleAttr.X)*cargos.get(i).getQuantity());
//					}
//				}
//			}
//		}
//	return counter;
//	}
	
	private BBNode[] selectBranching(ArrayList<Integer> nonBinaryVariables, BBNode node) throws Exception{
		double[] selection1 = branchingSelection(nonBinaryVariables, node);
		double[] selection2 = branchingSelectionCargo(nonBinaryVariables, node);
		double selection3 = branchingSelectionNumberOfCargoes();
		double[] selection4 = branchingSelectionArc(nonBinaryVariables, node);
		double[] selection5 = branchingSelectionArc2(nonBinaryVariables, node);
		
//		System.out.println("num vechicles = " +this.numVehicles.get(GRB.DoubleAttr.X));
//		this.convexity.print();
		if(this.numVehicles.get(GRB.DoubleAttr.X)-Math.floor(this.numVehicles.get(GRB.DoubleAttr.X))>0.001 && this.numVehicles.get(GRB.DoubleAttr.X)-Math.floor(this.numVehicles.get(GRB.DoubleAttr.X))<1-0.001) {
			return branchOnNumVehicles(node, (int) Math.floor(this.numVehicles.get(GRB.DoubleAttr.X))); 	
			}
//			else {
//				if(selection3-Math.floor(selection3)>0.001 && selection3-Math.floor(selection3)<1-0.001) {
//				System.out.println("Branching on Number of Cargoes with sum "+selection3);
//				return branchOnNumberOfCargo(node, (int) selection3);
//				}
//			}
//		}
//		else{
//			if(Math.abs(selection2[1]-Math.floor(selection2[1])-0.5)<Math.abs(selection3-Math.floor(selection3)-0.5)) {
//				if(selection2[1]-Math.floor(selection2[1])>0.001 && selection2[1]-Math.floor(selection2[1])<1-0.001) {
//				int[] branch = {(int)selection2[0], (int)selection2[1]};
//				System.out.println("Variable Cargo branch is "+selection2[0]+" with sum "+ selection2[1]);
//				return branchOnCargo(node, branch);
//				}
//			}
//			else {
//				if(selection3-Math.floor(selection3)>0.001 && selection3-Math.floor(selection3)<1-0.001) {
//					System.out.println("Branching on Number of Cargoes with sum "+selection3);
//					return branchOnNumberOfCargo(node, (int) selection3);
//				}
//			}
//			
//		}
//		if(selection3-Math.floor(selection3)>0.01 && selection3-Math.floor(selection3)<1-0.01) {
//			System.out.println("Branching on Number of Cargoes with sum "+selection3);
//			return branchOnNumberOfCargo(node, (int) selection3);
//			}
		
		
		
		
		
		else 
			if(selection2[1]-Math.floor(selection2[1])>0.001 && selection2[1]-Math.floor(selection2[1])<1-0.001) {
			int[] branch = {(int)selection2[0], (int)selection2[1]};
			System.out.println("Variable Cargo branch is "+selection2[0]+" with sum "+ selection2[1]);
			return branchOnCargo(node, branch);
		}
		else if(selection4[2]-Math.floor(selection4[2])>0.001 && selection4[2]-Math.floor(selection4[2])<1-0.001) {
			int[] branch = {(int)Math.rint(selection4[0]),(int)Math.rint(selection4[1]),(int)Math.floor(selection4[2])};
			System.out.println("Variable arc branch is "+branch[0]+" "+branch[1]+" with sum "+ selection4[2]);
//				System.exit(0);
			return branchOnArc(node, branch);
		}
		else 
		if(selection5[3]-Math.floor(selection5[3])>0.001 && selection5[3]-Math.floor(selection5[3])<1-0.001) {
			int[] branch = {(int)Math.rint(selection5[0]),(int)Math.rint(selection5[1]),(int)Math.rint(selection5[2]),(int)Math.floor(selection5[3])};
			System.out.println("Variable arc branch is "+branch[0]+" "+branch[1]+" with sum "+ selection4[2]);
//				System.exit(0);
			return branchOnTwoArc(node, branch);
		}
//		else 
//			if(selection1[2]>0.002 && selection1[2]<1-0.002) {
//			int[] branch = {(int)selection1[0],(int)selection1[1]};
//			System.out.println("Variable branch is "+selection1[0]+" "+selection1[1]+" with sum "+ selection1[2]);
//			return branchOnCargoAndShip(node, branch);
//		}
//		else if(selection4[3]>0.001 && selection4[3]<1-0.001) {
//			int[] branch = {(int)Math.rint(selection4[0]),(int)Math.rint(selection4[1]),(int)Math.rint(selection4[2])};
//			System.out.println("Variable arc branch is "+branch[0]+" "+branch[1]+" "+branch[2]+" with sum "+ selection4[3]);
////			System.exit(0);
//			return branchOnArc(node, branch);
//		}
		
		return null;
	}
	
	public int generateTwoPathCuts(ArrayList<Integer> nonZeroPaths) throws Exception{
		
	
		Date d1 = new Date();
		two.RouteBasedSeparationAlgorithm(this.getTwoCutPaths(nonZeroPaths));
		Date d2 = new Date();
		System.out.println("Sep 1 finished: "+(d2.getTime()-d1.getTime()));
		two.getTwoPathCutSubsets();
		Date d3 = new Date();
		System.out.println("Sep 2 finished: "+(d3.getTime()-d2.getTime()));
//		ArrayList<Vector<Integer>> subsets3 = capcuts.getCapacityCutSubsets(arcflow, yvars, two.subsets);
//		subsets.addAll(subsets2);
//		subsets.addAll(subsets3);
//		two.subsets.addAll(c)
		
		int size = two.newsubsets.size();
		for(Vector<Integer> vector : two.newsubsets) {
			String temp = "";
			System.out.println(vector.toString());
//			GRBLinExpr expr = new GRBLinExpr();
			GRBConstr con = model.addConstr(new GRBLinExpr(), GRB.GREATER_EQUAL, two.getRightHandSide(vector), "two path cut "+vector.toString());
			model.update();
//			con.set(attr, newvalue)
//			con.setType(XPRB.G);
//			con.add(two.getRightHandSide(vector));
//			con.setModCut(true);
//			int yvalue = two.Yval.get(vector);
			for(int i : vector) {
				if(i<cargos.size()) {
//					System.out.println(vector.size());
//					double temp = -2.0/(vector.size());
//					System.out.println(temp);
					int ple = -two.getYcoefficient(vector, i, two.RHS.get(vector));
					model.chgCoeff(con,optionalVars[i], ple);
					temp+="("+i+" "+ple+") ";
					System.out.println("adding "+i+" with coeff "+temp);
				}
				else if(!vector.contains(i-cargos.size())){
					int ple = -two.getYcoefficient(vector, i-cargos.size(), two.RHS.get(vector));
//					double temp = -(2.0/vector.size());
					model.chgCoeff(con,optionalVars[i-cargos.size()], ple);
					temp+="("+i+" "+ple+") ";
				}
			}
			for(GRBVar var : variables) {
				Path path = pathList.get(allPaths.get(var));
				int counter = 0;
				int previous = path.portList.get(1).getId();
				for(int i = 2; i < path.portList.size(); i++) {
					int current = path.portList.get(i).getId();
					if(vector.contains(previous) && !vector.contains(current)) {
						counter++;
					}
					previous = current;
				}
				model.chgCoeff(con,var, counter);
			}
			temp+= "RHS: "+two.getRightHandSide(vector);
//			file.writeTestOutput(con.toString()+" "+temp);
//			file.flush();
//			con.assign(expr.gEql(two.getRightHandSide(vector)));
			
//			twopathcutsSubsets.add(vector);
			cutConstraints.put(vector, con);
//			cutexpr.put(vector, expr);
			
		}
		two.resetNewSubsets();
		return size;
	}
	
//	public int generateCapacityCuts(double[][] arcflow, double[] yvars, CapacityCuts cap) {
//		ArrayList<Vector<Integer>> subsets = cap.getCapacityCutSubsets(arcflow, yvars, capcutsSubsets);
//		
//		for(Vector<Integer> vector : subsets) {
////			GRBLinExpr expr = new GRBLinExpr();
//			GRBConstr con = problem.newCtr("capacity cut "+vector.toString());
//			con.setType(XPRB.G);
//			con.add(cap.getRightHandSide(vector));
//			con.setModCut(true);
//			for(int i : vector) {
//				if(i<cargos.size()) {
////					System.out.println(vector.size());
////					double temp = -2.0/(vector.size());
////					System.out.println(temp);
//					con.addTerm(optionalVars[i], -2);
////					System.out.println("adding "+i+" with coeff "+temp);
//				}
//				else if(!vector.contains(i-cargos.size())){
////					double temp = -(2.0/vector.size());
//					con.addTerm(optionalVars[i-cargos.size()], -2);
//				}
//			}
//			for(GRBVar var : variables) {
//				Path path = pathList.get(allPaths.get(var));
//				int counter = 0;
//				int previous = path.portList.get(1).getId();
//				for(int i = 2; i < path.portList.size(); i++) {
//					int current = path.portList.get(i).getId();
//					if(vector.contains(previous) && !vector.contains(current)) {
//						counter++;
//					}
//					previous = current;
//				}
//				con.addTerm(var, counter);
//			}
//			
////			con.assign(expr.gEql(cap.getRightHandSide(vector)));
//			
//			capcutsSubsets.add(vector);
//			capcutConstraints.put(vector, con);
////			capcutexpr.put(vector, expr);
//		}
//		return subsets.size();
//	}
	
	private void setArcDualValues() throws Exception{
		double[][] arcdualvalues = new double[cargos.size()*2][cargos.size()*2+1];
		double[] firstarcdualvalues = new double[cargos.size()];
		double[][][] twoArcdualvalues = new double[cargos.size()*2][cargos.size()*2][cargos.size()*2+1];
		for(Vector<Integer> subset : two.subsets) {
//			Vector<Integer> subset = cutConstraints.get(con);
			GRBConstr con = cutConstraints.get(subset);
			for(int i : subset) {
//				System.out.println("should not be here!");
				for(int j = 0; j < cargos.size()*2+1; j++) {
					if(!subset.contains(j)) {
						arcdualvalues[i][j] += con.get(GRB.DoubleAttr.Pi);
					}
				}
			}
		}
		for(Vector<Integer> subset : capcutsSubsets) {
//			Vector<Integer> subset = cutConstraints.get(con);
			GRBConstr con = capcutConstraints.get(subset);
			for(int i : subset) {
				for(int j = 0; j < cargos.size()*2+1; j++) {
					if(!subset.contains(j)) {
						arcdualvalues[i][j] += con.get(GRB.DoubleAttr.Pi);
					}
				}
			}
		}
		
		for(int i = 0; i < cargos.size(); i++) {
			if(firstarcBranches[0][i]!= null) {
				firstarcdualvalues[i]+=firstarcBranches[0][i].get(GRB.DoubleAttr.Pi);
			}
			if(firstarcBranches[1][i]!= null) {
				firstarcdualvalues[i]+=firstarcBranches[1][i].get(GRB.DoubleAttr.Pi);
			}
		}
		
		
		for(int i = 0; i < cargos.size()*2; i++) {
			for(int j = 0; j < cargos.size()*2+1; j++) {
				if(arcBranches[0][i][j]!= null) {
					arcdualvalues[i][j]+=arcBranches[0][i][j].get(GRB.DoubleAttr.Pi);
				}
				if(arcBranches[1][i][j]!= null) {
					arcdualvalues[i][j]+=arcBranches[1][i][j].get(GRB.DoubleAttr.Pi);
				}
			}
		}
		
		for(int i = 0; i < cargos.size()*2; i++) {
			for(int j = 0; j < cargos.size()*2; j++) {
				for(int k = 0; k < cargos.size()*2+1; k++) {
					if(twoArcBranches[0][i][j][k]!= null) {
						twoArcdualvalues[i][j][k]+=twoArcBranches[0][i][j][k].get(GRB.DoubleAttr.Pi);
//						System.out.println("two arc dual value "+i+" "+j+" "+k+" "+twoArcBranches[0][i][j][k].get(GRB.DoubleAttr.Pi));
					}
					if(twoArcBranches[1][i][j][k]!= null) {
						twoArcdualvalues[i][j][k]+=twoArcBranches[1][i][j][k].get(GRB.DoubleAttr.Pi);
//						System.out.println("two arc dual value "+i+" "+j+" "+k+" "+twoArcBranches[1][i][j][k].get(GRB.DoubleAttr.Pi));
					}
				}
			}
		}
		route.setArcdualvalues(arcdualvalues, firstarcdualvalues, twoArcdualvalues);
		route2.setArcdualvalues(arcdualvalues, firstarcdualvalues, twoArcdualvalues);
		route3.setArcdualvalues(arcdualvalues, firstarcdualvalues, twoArcdualvalues);
	}
	
	private void setShipCargoDuals() throws Exception{
		double[][] nonOverFullGroupDuals = new double[ships.size()][cargos.size()];
		double[][] shipCargoDuals = new double [ships.size()][cargos.size()];
		double[][] shipCargoDuals2 = new double [ships.size()][cargos.size()];
		double[][][] shipCargoGroupDuals = new double [ships.size()][cargos.size()][cargos.size()];
		for(int i = 0; i < cargos.size(); i++) {
			for(int s = 0; s < ships.size(); s++) {
//				if(nonOverFullGroupCuts2[i]!= null) {
//					nonOverFullGroupDuals[s][i]-=(nonOverFullGroupCuts2[i].getDual()*(ships.size()-1));
//				}
				if(nonOverFullGroupCuts3[i]!= null) {
					nonOverFullGroupDuals[s][i]-=(nonOverFullGroupCuts3[i].get(GRB.DoubleAttr.Pi));
				}
			}
		}
//		
//		System.out.println(Arrays.toString(nonOverFullGroupDuals[0]));
		for(int s = 0; s < ships.size(); s++) {
			for(int i = 0; i < cargos.size(); i++) {
//				if(this.nonOverFullGroupCuts[s][i]!=null) {
////					nonOverFullGroupCuts[s][i].print();
////					System.out.println(s+" "+i+" "+nonOverFullGroupCuts[s][i].getDual());
//					for(int s2 = 0 ; s2 < ships.size(); s2++) {
//						if(s==s2) {
//							nonOverFullGroupDuals[s][i]-=(nonOverFullGroupCuts[s][i].getDual()*(ships.size()-1));
//						}
//						else {
//							shipCargoDuals2[s2][i]-=nonOverFullGroupCuts[s][i].getDual();
//						}
//					}
//				}
//				
//				
				for(int j = i+1; j < cargos.size(); j++) {
//					if(this.overFullGroupCuts[i][j]!=null) {
//						for(int s2 = 0 ; s2 < ships.size(); s2++) {
//							shipCargoGroupDuals[s2][i][j]+=this.overFullGroupCuts[i][j].getDual();
//						}
//					}
//					if(this.overFullGroupCuts2[i][j]!=null) {
////						if(this.overFullGroupCuts2[i][j].getDual()>tm.getZeroTol()) {
////							System.out.println(i+" "+j+" "+this.overFullGroupCuts2[i][j].getDual());
////							System.exit(0);
////						}
//						for(int s2 = 0 ; s2 < ships.size(); s2++) {
//							shipCargoGroupDuals[s2][i][j]+=this.overFullGroupCuts2[i][j].getDual()*2;
//						}
//					}
					if(this.shipCargoCuts[s][i][j]!=null) {
						for(int s2 = 0 ; s2 < ships.size(); s2++) {
							if(s==s2) {
	//							if(s==1 && i == 1 && (j ==3 || j == 9)) {
	////								System.out.println("Dual "+this.shipCargoCuts[s][i][j].getDual());
	////								shipCargoCuts[s][i][j].print();
	//							}
								shipCargoGroupDuals[s2][i][j]+=this.shipCargoCuts[s][i][j].get(GRB.DoubleAttr.Pi);
							}
							else {
	//							if(s2==1 && i == 1 && (j ==3 || j == 9)) {
	//								System.out.println("Other Dual "+this.shipCargoCuts[s][i][j].getDual());
	//							}
								shipCargoDuals[s2][i]-=this.shipCargoCuts[s][i][j].get(GRB.DoubleAttr.Pi);
								shipCargoDuals[s2][j]-=this.shipCargoCuts[s][i][j].get(GRB.DoubleAttr.Pi);
							}
						}
					}
				}
			}
		}
		route.setShipCargoDualvalues(shipCargoDuals);
		route.setShipCargoGroupDualvalues(shipCargoGroupDuals);
		route.setNonOverFullGroupDualvalues(nonOverFullGroupDuals);
		route.setShipCargoDualvalues2(shipCargoDuals2);
		route2.setShipCargoDualvalues(shipCargoDuals);
		route2.setShipCargoGroupDualvalues(shipCargoGroupDuals);
		route2.setNonOverFullGroupDualvalues(nonOverFullGroupDuals);
		route2.setShipCargoDualvalues2(shipCargoDuals2);
		route3.setShipCargoDualvalues(shipCargoDuals);
		route3.setShipCargoGroupDualvalues(shipCargoGroupDuals);
		route3.setNonOverFullGroupDualvalues(nonOverFullGroupDuals);
		route3.setShipCargoDualvalues2(shipCargoDuals2);
	}
	
	private void addPathToCuts(GRBVar var, Path path) throws Exception{
		for(Vector<Integer> vector : two.subsets) {
//			Vector<Integer> subset = cutConstraints.get(con);
			GRBConstr con = cutConstraints.get(vector);
		int counter = 0;
		int previous = path.portList.get(1).getId();
		for(int i = 2; i < path.portList.size(); i++) {
			int current = path.portList.get(i).getId();
			if(vector.contains(previous) && !vector.contains(current)) {
				counter++;
			}
			previous = current;
		}
		
//		con.print();
		model.chgCoeff(con,var, counter);
//		if(counter>0) {
//		System.out.println("should have added "+var.getName());
////		con.assign(null);
//		con.print();
//		System.exit(0);
//		}
		}
		for(int s = 0; s < ships.size(); s++) {
			for(int i = 0; i < cargos.size(); i++) {
				for(int j = i+1; j < cargos.size(); j++) {
//					if(this.overFullGroupCuts[i][j]!=null) {
//						if(checkForSet(i, j, path)) {
//							this.overFullGroupCuts[i][j].addTerm(var,1);
//						}
//					}
//					if(this.overFullGroupCuts2[i][j]!=null) {
//						if(checkForSet(i, j, path)) {
//							this.overFullGroupCuts2[i][j].addTerm(var,2);
//						}
//					}
					if(this.shipCargoCuts[s][i][j]!=null) {
//						
							if(ships.indexOf(path.ship)==s && checkForSet(i, j, path)) {
								model.chgCoeff(this.shipCargoCuts[s][i][j],var, -1);
							}
							else if(ships.indexOf(path.ship)!=s){
								int ple = 0;
								if(path.cargosHandled.contains(i)) {
									ple++;
								}
								if(path.cargosHandled.contains(j)) {
									ple++;
								}
								if(ple>0) {
////									int ple = path.cargosHandled. + path.getCargoesHandled()[j];
									model.chgCoeff(this.shipCargoCuts[s][i][j],var, ple);
								}
							}
						}
					}
				}
		}
		
//		for(int s = 0; s < ships.size(); s++) {
//			for(int i = 0; i < cargos.size(); i++) {
//				if(this.nonOverFullGroupCuts[s][i]!= null) {
//					if(ships.indexOf(path.ship)==s) {
//						if(path.cargosHandled.contains(i) && checkForNonOverFullGroup(i, path)) {
//							this.nonOverFullGroupCuts[s][i].addTerm(var,(ships.size()-1));
////							System.out.println("adding "+path.toString()+" to cut for "+s+" "+i);
//						}
//					}
//					else {
//						if(path.cargosHandled.contains(i)) {
//							this.nonOverFullGroupCuts[s][i].addTerm(var,1);
//						}
//					}
//				}
//			}
//		}
		for(int i = 0; i < cargos.size(); i++) {
			if(path.cargosHandled.contains(i) && checkForNonOverFullGroup(i, path)) {
//				if(this.nonOverFullGroupCuts2[i]!=null) {
//					this.nonOverFullGroupCuts2[i].addTerm(var, (ships.size()-1));
//				}
				if(this.nonOverFullGroupCuts3[i]!= null) {
					model.chgCoeff(this.nonOverFullGroupCuts3[i],var, 1);
				}
			}
		}
	}
	private void addPathToCapCuts(GRBVar var, Path path) throws Exception{
		for(Vector<Integer> vector : capcutsSubsets) {
//			Vector<Integer> subset = cutConstraints.get(con);
			GRBConstr con = capcutConstraints.get(vector);
		int counter = 0;
		int previous = path.portList.get(1).getId();
		for(int i = 2; i < path.portList.size(); i++) {
			int current = path.portList.get(i).getId();
			if(vector.contains(previous) && !vector.contains(current)) {
				counter++;
			}
			previous = current;
		}
		
//		con.print();
		model.chgCoeff(con,var, counter);
//		System.out.println("should have added "+var.getName());
//		con.assign(null);
//		con.print();
		}
	}
	
	private void initiateTwoPathCuts() throws Exception{
		for(Vector<Integer> vector : twopathcutsSubsets) {
//			GRBLinExpr expr = new GRBLinExpr();
			GRBConstr con = model.addConstr(new GRBLinExpr(), GRB.GREATER_EQUAL,two.getRightHandSide(vector), "");
//			con.setType(XPRB.G);
//			con.add(two.getRightHandSide(vector));
//			con.setModCut(true);
			for(int i : vector) {
				if(i<cargos.size()) {
//					System.out.println(vector.size());
//					double temp = -2.0/(vector.size());
//					System.out.println(temp);
					model.chgCoeff(con,optionalVars[i], -1);
//					System.out.println("adding "+i+" with coeff "+temp);
				}
				else if(!vector.contains(i-cargos.size())){
//					double temp = -(2.0/vector.size());
					model.chgCoeff(con,optionalVars[i-cargos.size()], -1);
				}
			}
			
			
//			con.assign(expr.gEql(two.getRightHandSide(vector)));
			
//			twopathcutsSubsets.add(vector);
//			cutexpr.put(vector, expr);
			cutConstraints.put(vector, con);
		}
	}
	private void initiateCapacityCuts() throws Exception{
		for(Vector<Integer> vector : capcutsSubsets) {
			GRBConstr con = model.addConstr(new GRBLinExpr(), GRB.GREATER_EQUAL, capcuts.getRightHandSide(vector), "");
//			con.setType(XPRB.G);
//			con.add(capcuts.getRightHandSide(vector));
//			con.setModCut(true);
			for(int i : vector) {
				if(i<cargos.size()) {
//					System.out.println(vector.size());
//					double temp = -2.0/(vector.size());
//					System.out.println(temp);
					model.chgCoeff(con,optionalVars[i], -2);
//					System.out.println("adding "+i+" with coeff "+temp);
				}
				else if(!vector.contains(i-cargos.size())){
//					double temp = -(2.0/vector.size());
					model.chgCoeff(con,optionalVars[i-cargos.size()], -2);
				}
			}
			
//			GRBConstr con = problem.newCtr();
//			con.assign(expr.gEql(capcuts.getRightHandSide(vector)));
//			con.setModCut(true);
//			twopathcutsSubsets.add(vector);
//			capcutexpr.put(vector, expr);
			capcutConstraints.put(vector, con);
		}
	}
	
	private void setUpperAndLowerBound(BBNode node) throws Exception{
		
		
		
			int[] cargoBranches = new int[tm.getCargos().size()];
			int[][] arcmatrix = new int[tm.getCargos().size()*2][tm.getCargos().size()*2+1];
			int[] firstarc = new int[cargos.size()];
			int[] cargoUpperLimit = new int[cargos.size()]; 
//			Arrays.fill(cargoUpperLimit, 2);
			int[] fullcargo = new int[cargos.size()];
			for(int b : node.getCargoBranches()) {
				if(tm.getCargoBranches().get(b).getDecision()==-1) {
					cargoUpperLimit[tm.getCargoBranches().get(b).getCargo()] = tm.getCargoBranches().get(b).getValue();
				}
				if(tm.getCargoBranches().get(b).getDecision()==-1 && tm.getCargoBranches().get(b).getValue()==1) {
					fullcargo[tm.getCargoBranches().get(b).getCargo()] = 1;
				}
				
				if(tm.getCargoBranches().get(b).getValue()<=0 && tm.getCargoBranches().get(b).getDecision()==-1) {
					cargoBranches[tm.getCargoBranches().get(b).getCargo()]=-1;
				}
			}
			
			for(int b : node.getBranches()) {
				if(tm.getBranches().get(b) instanceof Arcbranch) {
					Arcbranch arc = (Arcbranch) tm.getBranches().get(b);
					if(arc.decision==-1 && arc.value<= 0) {
						if(arc.cargo==-1) {
							firstarc[arc.head] = -1;
						}
						else {
							arcmatrix[arc.cargo][arc.head] = -1;
						}
				}
				}
			}
//			ArrayList<GRBVar> removedVars = new ArrayList<GRBVar>();
			for(GRBVar var : variables) {
				boolean fixed = false;
				Path p = pathList.get(allPaths.get(var));
				int[] cargoCounter = new int[cargos.size()];
			
					int current = -1;
					int previous = p.portList.get(1).getId();
					double fullCargo = 0;
					if(previous<cargos.size()) {
						cargoCounter[previous]++;
						if(cargoUpperLimit[previous]==1) {
							fullCargo += cargos.get(previous).getQuantity();
							if(fullCargo-tm.getZeroTol()>ships.get(0).getOriginalVolumeCapacity()) {
								var.set(GRB.DoubleAttr.LB,0);
								var.set(GRB.DoubleAttr.UB,0);
								fixed = true;
								break;
							}
						}
					}
//					if(p.portList.get(1).getId()<cargos.size() && firstarc[p.portList.get(1).getId()]==-1) {
//						var.set(GRB.DoubleAttr.LB,0);
//						var.set(GRB.DoubleAttr.UB,0);
//						fixed = true;
////						break;
//					}
					
					for(int i = 2; i < p.portList.size()-1; i++) {
						current = p.portList.get(i).getId();
						if(current < cargos.size()) {
							if(cargoUpperLimit[current]==1) {
								fullCargo += cargos.get(current).getQuantity();
								if(fullCargo-tm.getZeroTol()>ships.get(0).getOriginalVolumeCapacity()) {
									var.set(GRB.DoubleAttr.LB,0);
									var.set(GRB.DoubleAttr.UB,0);
									fixed = true;
									break;
								}
							}
						
							if(cargoBranches[current]==-1) {
								var.set(GRB.DoubleAttr.LB,0);
								var.set(GRB.DoubleAttr.UB,0);
								fixed = true;
								break;
							}
						}
						else if(current < cargos.size()*2) {
							if(cargoUpperLimit[current-cargos.size()]==1) {
								fullCargo -= cargos.get(current-cargos.size()).getQuantity();
							}
						}
//						else if(arcmatrix[previous][current]==-1) {
//							var.fix(0);
//							fixed = true;
//							break;
//						}
						previous = current;
						if(previous<cargos.size()) {
							cargoCounter[previous]++;
						}
					}
					for(int i = 0; i < cargos.size(); i++) {
						
						
						if((cargoUpperLimit[i]>=1 && cargoCounter[i]>=cargoUpperLimit[i]+1)) {// || (cargoCounter[i]>=1 && fullcargo[i]==1 && p.cargoVolume[i]<=cargos.get(i).getQuantity()-1)) {
//							System.out.println("fixing: "+p.toString());
//							System.out.println(Arrays.toString(cargoCounter));
							var.set(GRB.DoubleAttr.LB,0);
							var.set(GRB.DoubleAttr.UB,0);
//							model.remove(var);
//							removedVars.add(var);
//							obj.remove(var);
							fixed = true;
							break;
						}
					}
				if(!fixed) {
					var.set(GRB.DoubleAttr.LB,0);
					var.set(GRB.DoubleAttr.UB,GRB.INFINITY);
				}
		}
//			variables.removeAll(removedVars);
//			model.update();
	}
	private ArrayList<TwoCutPath> getTwoCutPaths(ArrayList<Integer> nonZeroPaths) throws Exception{
		PriorityQueue<TwoCutPath> sortedPaths = new PriorityQueue<TwoCutPath>(10, new TwoCutPathComparator());
		for(int i : nonZeroPaths) {
			Path p = pathList.get(allPaths.get(variables.get(i)));
			TwoCutPath path = new TwoCutPath(p, variables.get(i).get(GRB.DoubleAttr.X), cargos);
			sortedPaths.add(path);
		}
		ArrayList<TwoCutPath> list = new ArrayList<TwoCutPath>();
		while(!sortedPaths.isEmpty()) {
			list.add(sortedPaths.poll());
		}
		return list;
	}
	
	public TwoPathCuts getTwoPathCuts() {
		return this.two;
	}
	
	private void addShipCargoCuts() throws Exception{
		for(int s = 0; s < ships.size(); s++) {
			for(int i = 0; i < cargos.size(); i++) {
				for(int j = i+1; j < cargos.size(); j++) {
				if(cargos.get(i).getQuantity()+cargos.get(j).getQuantity()>ships.get(s).getOriginalVolumeCapacity()) {
					
					GRBConstr con = model.addConstr(new GRBLinExpr(), GRB.GREATER_EQUAL, 0, "ship cargo cut "+s+" "+i+" "+j);
//					con.setType(XPRB.G);
//					con.add(0);
//					con.setModCut(true);
					this.shipCargoCuts[s][i][j] = con;
				}
				}
			}
		}
	}
	
//	private void addNonOverFullGroupCuts() {
//		for(int s = 0; s < ships.size(); s++) {
//			for(int i = 0; i < cargos.size(); i++) {
//					if(cargos.get(i).getQuantity()<ships.get(s).getOriginalVolumeCapacity()) {
//					
//					GRBConstr con = problem.newCtr("nonoverfullgroup "+s+" "+i);
//					con.setType(XPRB.L);
//					con.add((ships.size()-1));
//					con.setModCut(true);
//					this.nonOverFullGroupCuts[s][i] = con;
//				}
//				
//			}
//		}
//	}
	
	public boolean checkForSet(int i, int j, Path p) {
		if(!p.cargosHandled.contains(i) || !p.cargosHandled.contains(j)) {
			return false;
		}
		ArrayList<int[]> list = p.getDeliveryGroups2(1);
		for(int[] array : list) {
			if(array[i]>0 && array[j]>0) {
				return true;
			}
		}
		return false;
	}
	
	public boolean checkForNonOverFullGroup(int i, Path p) {
		if(!p.cargosHandled.contains(i)) {
			return false;
		}
		ArrayList<int[]> list = p.getDeliveryGroups2(1);
		for(int[] array : list) {
			if(array[i]>0) {
				double sum = 0;;
				for(int j = 0; j < cargos.size(); j++) {
					if(array[j]>0) {
						sum += cargos.get(j).getQuantity();
					}
				}
				if(sum>p.ship.getOriginalVolumeCapacity()+tm.getZeroTol()) {
					return false;
				}
			}
				
		}
		return true;
	}
	public int checkForDublicatePaths() {
		int counter = 0;
		for(int i = 0; i < pathList.size(); i++) {
			Path p1 = pathList.get(i);
			for(int j = i+1; j < pathList.size(); j++) {
				Path p2 = pathList.get(j);
				if(p1.getShip()==p2.getShip() &&  p1.getPortList().size()==p2.getPortList().size()) {
					boolean samePath = true;
					for(int k = 1; k < p1.getPortList().size(); k++) {
						if(p1.getPortList().get(k).getId()!=p2.getPortList().get(k).getId() 
								|| (p1.getPortList().get(k).getId() < cargos.size() 
									&&	p1.cargoVolume[p1.getPortList().get(k).getId()]!= p2.cargoVolume[p2.getPortList().get(k).getId()])) {
							samePath=false;
							break;
						}
					}
					if(samePath) {
						System.out.println(p1.toString());
						System.out.println(p2.toString());
						counter++;
					}
				}
			}
		}
		return counter;
	}
	private ArrayList<Vector<Integer>> getDeliveryGroups(Path path) {
		int FirstPickup = 1;
		ArrayList<Vector<Integer>> list = new ArrayList<Vector<Integer>>();
		Vector<Integer> temp = new Vector<Integer>();
		int teller = 0;
		int previous = -1;
//		System.out.println(FirstPickup);
//		for(int p = 0; p< path.portList.size(); p++) {
//			System.out.print(path.portList.get(p).getId()+",");
//		}
//		System.out.println();
		
		for(int p = FirstPickup; p< path.portList.size(); p++) {
//			System.out.println(path.portList.get(p).getId());
			if(path.portList.get(p).getId()<path.numberOfCargos) {
//				System.out.println(path.portList.get(p).getId()+" "+cargos.get(path.portList.get(p).getId()).getQuantityDual());
				
					temp.add(path.portList.get(p).getId());
					previous = path.portList.get(p).getId();
				
			}
			else if(path.portList.get(p).getId()<path.numberOfCargos*2) {
				
			
				if(previous<path.numberOfCargos) {
					list.add((Vector<Integer>)temp.clone());
				}
//				System.out.println(path.portList.get(p).getId()+" "+path.numberOfCargos);
//				System.out.println(temp.toString());
				temp.remove(temp.indexOf(path.portList.get(p).getId()-path.numberOfCargos));
				previous = path.portList.get(p).getId();
				}
			
//			previous = path.portList.get(p).getId();
		}
		return list;
	}
//	private void addNonOverFullGroupCuts2() {
//		
//		for(int i = 0; i < cargos.size(); i++) {
//				
//				GRBConstr con = problem.newCtr();
//				con.setType(XPRB.L);
//				con.add(ships.size());
//				con.setModCut(true);
//				con.addTerm(z[i],1);
//				this.nonOverFullGroupCuts2[i] = con;
//			}
//	}
	
//	private void addOverFullGroupCut(int i, int j) {
//		
//		GRBConstr con = problem.newCtr();
//		con.setType(XPRB.L);
//		con.add(0);
//		con.setModCut(true);
//		for(GRBVar var : variables) {
//			Path path = pathList.get(allPaths.get(var));
//			if(checkForSet(i, j, path) && cargos.get(i).getQuantity()+cargos.get(j).getQuantity()>path.ship.getOriginalVolumeCapacity()) {
//						con.addTerm(var, 1);
//			}
//		}
//		this.overFullGroupCuts[i][j] = con;
//	}
//private void addOverFullGroupCut2(int i, int j) {
//		
//		GRBConstr con = problem.newCtr();
//		con.setType(XPRB.L);
//		con.add(0);
//		con.setModCut(true);
//		con.addTerm(z[j],-1);
//		for(GRBVar var : variables) {
//			Path path = pathList.get(allPaths.get(var));
//			if(checkForSet(i, j, path) && cargos.get(i).getQuantity()+cargos.get(j).getQuantity()>path.ship.getOriginalVolumeCapacity()) {
//						con.addTerm(var, 2);
//			}
//		}
//		this.overFullGroupCuts2[i][j] = con;
//	}

	private void addSubsetRowCuts() {
		
		//int[] branchedOn = new int[cargos.size()];
		branchedOn = new Vector<Integer>();
//		for(int i = 0; i < cargos.size(); i++) {
//			if(z[i].getUB()==1.0) {
//				branchedOn.add(i);
//			}
//		}
////		System.out.println(branchedOn.toString());
//		subsetRowCuts = new GRBConstr[cargos.size()][cargos.size()][cargos.size()];
//		for( int i = 0; i < branchedOn.size(); i++) {
//			for( int j = i+1; j < branchedOn.size(); j++) {
//				for( int k = j+1; k < branchedOn.size(); k++) {
//					//if(branchedOn[i]+branchedOn[j]+branchedOn[k]>2.9){
//						tm.subsetRowCounter++;
//						GRBConstr con = problem.newCtr();
//						
//						con.add(1);
//						con.setType(XPRB.L);
//						populateSubsetRowCut(branchedOn.get(i),branchedOn.get(j),branchedOn.get(k), con);
//						subsetRowCuts[branchedOn.get(i)][branchedOn.get(j)][branchedOn.get(k)] = con;
//					//}
//				}
//			}
//		}
	}	
//		double[][][] flow = new double[cargos.size()][cargos.size()][cargos.size()];
//		for(GRBVar var : variables) {
//			if(var.get(GRB.DoubleAttr.X)>tm.getZeroTol()) {
//				Path p = pathList.get(allPaths.get(var));
//				for( int i = 0; i < cargos.size(); i++) {
//					for( int j = i+1; j < cargos.size(); j++) {
//						for( int k = j+1; k < cargos.size(); k++) {
//							if(subsetRowCuts[i][j][k]==1 && p.cargosHandled[i]+p.cargosHandled[j]+p.cargosHandled[k]>=1.5) {
//								flow[i][j][k]+=var.get(GRB.DoubleAttr.X);
//					}
//				}
//					}
//				}
//			}
//		}
//		
//		for( int i = 0; i < cargos.size(); i++) {
//			for( int j = i+1; j < cargos.size(); j++) {
//				for( int k = j+1; k < cargos.size(); k++) {
//					if(branchedOn[i]+branchedOn[j]+branchedOn[k]>2.9) {
//						if(flow[i][j][k]>1.1) {
//							tm.subsetRowCounter++;
//							tm.subsetRowFlows.add(flow[i][j][k]);
//						}
//					}
//				}
//			}
//		}
//	}
	
	
	
//	private void populateSubsetRowCut(int i, int j, int k, GRBConstr con) {
//		for(GRBVar var : variables) {
//			Path p = pathList.get(allPaths.get(var));
//			int counter = 0;
//			if(p.cargosHandled.contains(i)) {
//				counter++;
//			}
//			if(p.cargosHandled.contains(j)) {
//				counter++;
//			}
//			if(p.cargosHandled.contains(k)) {
//				counter++;
//			}
//			if(counter>1.5) {
//				con.addTerm(var,1);
//			}
//		}
//	}
//	
//	private void addVarToSubsetRowCut(GRBVar var, Path p) {
//		for( int i = 0; i < branchedOn.size(); i++) {
//			for( int j = i+1; j < branchedOn.size(); j++) {
//				for( int k = j+1; k < branchedOn.size(); k++) {
//					if(subsetRowCuts[branchedOn.get(i)][branchedOn.get(j)][branchedOn.get(k)]!=null) {
//						int counter = 0;
//						if(p.cargosHandled.contains(i)) {
//							counter++;
//						}
//						if(p.cargosHandled.contains(j)) {
//							counter++;
//						}
//						if(p.cargosHandled.contains(k)) {
//							counter++;
//						}
//						if(counter>1.5) {
//							subsetRowCuts[branchedOn.get(i)][branchedOn.get(j)][branchedOn.get(k)].addTerm(var,1);
//						}
//					}
//				}
//			}
//		}
//	}
//	private void setSubsetRowDuals() {
//		
//		double[][][] duals = new double[cargos.size()][cargos.size()][cargos.size()];
//		for( int i = 0; i < branchedOn.size(); i++) {
//			for( int j = i+1; j < branchedOn.size(); j++) {
//				for( int k = j+1; k < branchedOn.size(); k++) {
//					if(subsetRowCuts[branchedOn.get(i)][branchedOn.get(j)][branchedOn.get(k)]!=null) {
//						duals[branchedOn.get(i)][branchedOn.get(j)][branchedOn.get(k)]=subsetRowCuts[branchedOn.get(i)][branchedOn.get(j)][branchedOn.get(k)].getDual();
////						System.out.println("setting subsetrowduals "+branchedOn.get(i)+" "+branchedOn.get(j)+" "+branchedOn.get(k)+" to "+subsetRowCuts[branchedOn.get(i)][branchedOn.get(j)][branchedOn.get(k)].getDual());
//					}
//					else {
//						duals[branchedOn.get(i)][branchedOn.get(j)][branchedOn.get(k)]=0;
//					}
//				
//				}
//			}
//		}
//		
////		route.setSubSetRowsDuals(duals, branchedOn);
//	}
//	private void delSubsetRowCuts() {
//		for( int i = 0; i < branchedOn.size(); i++) {
//			for( int j = i+1; j < branchedOn.size(); j++) {
//				for( int k = j+1; k < branchedOn.size(); k++) {
//					if(subsetRowCuts[branchedOn.get(i)][branchedOn.get(j)][branchedOn.get(k)]!=null) {
//						problem.delCtr(subsetRowCuts[branchedOn.get(i)][branchedOn.get(j)][branchedOn.get(k)]);
//						subsetRowCuts[branchedOn.get(i)][branchedOn.get(j)][branchedOn.get(k)]=null;
//					}
//				}
//			}
//		}
//		subsetRowCuts = null;
//	}
	private ArrayList<Path> generateNewDelPatterns() {
		try{
		for(int i = 0; i < ships.size(); i++) {
			route3.transformCostMatrix(ships.get(i), cargos);
		}
		ArrayList<Path> paths = new ArrayList<Path>();
		for(GRBVar var : variables) {
			if(var.get(GRB.DoubleAttr.UB)>tm.getZeroTol()){ //&& var.getRCost()> -tm.getZeroTol()) {
			Path p = pathList.get(allPaths.get(var));
			if(p.ship.index>=0 && p.portList.size()>2){  //&& p.fullProb) {
				Path newp = route3.recalcPath(p);
				if(newp != null) {
					if(!checkForSame(newp, paths))
					paths.add(newp);
					//System.out.println(p.toString());
					//System.out.println(newp.toString());
					//System.out.println("-----------------");
				}
			}
			}
		}
		return paths;
		}
		catch(Exception e) {
			e.printStackTrace();
			System.exit(0);
			return null;
		}
//		for(int i = 0; i < paths.size(); i++) {
//			System.out.println(paths.get(i).toString());
//		}
//		if(paths.size()>0) {
//			System.exit(0);
//		}
		
	}
	
	private boolean checkForSame(Path p, ArrayList<Path> paths) {
		
		for(Path temppath : paths) {
			
		if(p.ship.getName().equalsIgnoreCase(temppath.ship.getName()) && p.portList.size() == temppath.portList.size()) {
			boolean found = true;
			for(int i = 1; i < p.portList.size(); i++) {
				if(p.getPortList().get(i).getId()!= temppath.portList.get(i).getId()) {
					found = false;
					break;
				}
			}
			if(found == true) {
				return true;
			}
		}
	}
		return false;
	}
	
	private void copyNode(BBNode parent, BBNode child1, BBNode child2) {
		Vector<Integer> branchesleft = new Vector<Integer>(parent.getBranches());
		Vector<Integer> cargobranchesleft = new Vector<Integer>(parent.getCargoBranches());
		
		
		Vector<Integer> cargobranchesright = new Vector<Integer>(parent.getCargoBranches());
		Vector<Integer> branchesright = new Vector<Integer>(parent.getBranches());
		
		
		child1.setBranches(branchesleft);
		child1.setCargoBranches(cargobranchesleft);
		child2.setBranches(branchesright);
		child2.setCargoBranches(cargobranchesright);
		child1.minVehicles = parent.minVehicles;
		child1.maxVehicles = parent.maxVehicles;
		child2.minVehicles = parent.minVehicles;
		child2.maxVehicles = parent.maxVehicles;
	}
	
	private boolean fixBranching(BBNode node) throws Exception{
		Date date1 = new Date();
		for(int i = 0; i < cargos.size(); i++) {
			z[i].set(GRB.DoubleAttr.LB, 0);
			z[i].set(GRB.DoubleAttr.UB, GRB.INFINITY);
			optionalVars[i].set(GRB.DoubleAttr.LB, 0);
			optionalVars[i].set(GRB.DoubleAttr.UB, 1);
			if(cargos.get(i).getOptional()==0) {
				optionalVars[i].set(GRB.DoubleAttr.LB, 1);
				optionalVars[i].set(GRB.DoubleAttr.UB, 1);
			}
		}
		model.update();
		System.out.println("max vehicles: "+node.maxVehicles);
		System.out.println("min vehicles: "+node.minVehicles);
		numVehicles.set(GRB.DoubleAttr.LB, 0);
		numVehicles.set(GRB.DoubleAttr.UB, GRB.INFINITY);
		numVehicles.set(GRB.DoubleAttr.LB, node.minVehicles);
		numVehicles.set(GRB.DoubleAttr.UB, node.maxVehicles);
		model.update();
//			this.numberOfCargoes.setUB(1000);
//			this.numberOfCargoes.setLB(0);
//			this.numberOfCargoes.setUB(node.getNumberOfCargoesBranchUpper());
//			this.numberOfCargoes.setLB(node.getNumberOfCargoesBranchLower());
		
			for(int cb : node.getCargoBranches()) {
				System.out.println("cargo branch: "+tm.getCargoBranches().get(cb).getCargo()+"  "+tm.getCargoBranches().get(cb).getValue()+"  "+tm.getCargoBranches().get(cb).getDecision());
//				}
				if(tm.getCargoBranches().get(cb).getValue()>=1
						&& tm.getCargoBranches().get(cb).getDecision()==1){
					optionalVars[tm.getCargoBranches().get(cb).getCargo()].set(GRB.DoubleAttr.LB,1);
					model.update();
				}
				if(tm.getCargoBranches().get(cb).getDecision()==-1){
					z[tm.getCargoBranches().get(cb).getCargo()].set(GRB.DoubleAttr.UB,Math.min(tm.getCargoBranches().get(cb).getValue(),z[tm.getCargoBranches().get(cb).getCargo()].get(GRB.DoubleAttr.UB)));
					model.update();
					if(tm.getCargoBranches().get(cb).getValue()==1 && cargos.get(tm.getCargoBranches().get(cb).getCargo()).getQuantity()>ships.get(0).getOriginalVolumeCapacity()+tm.getZeroTol()) {
						z[tm.getCargoBranches().get(cb).getCargo()].set(GRB.DoubleAttr.UB,0);
						model.update();
					}
				}
				else if(tm.getCargoBranches().get(cb).getDecision()==1){
					z[tm.getCargoBranches().get(cb).getCargo()].set(GRB.DoubleAttr.LB,Math.max(tm.getCargoBranches().get(cb).getValue(),z[tm.getCargoBranches().get(cb).getCargo()].get(GRB.DoubleAttr.LB)));
					model.update();
					if(tm.getCargoBranches().get(cb).getValue()==1 && cargos.get(tm.getCargoBranches().get(cb).getCargo()).getQuantity()>ships.get(0).getOriginalVolumeCapacity()+tm.getZeroTol()) {
						z[tm.getCargoBranches().get(cb).getCargo()].set(GRB.DoubleAttr.LB,Math.max(2,z[tm.getCargoBranches().get(cb).getCargo()].get(GRB.DoubleAttr.LB)));
						model.update();
					}
				}
			}
			model.update();
			boolean arcBranchExists = false;
			boolean twoarcBranchExists = false;
			for(int b : node.getBranches()) {
			
				if(tm.getBranches().get(b) instanceof Arcbranch) {
					arcBranchExists=true;
					Arcbranch arc = (Arcbranch) tm.getBranches().get(b);
					System.out.println("Arcbranch "+arc.cargo+" "+arc.head+" "+arc.decision+" "+arc.value);
					GRBConstr con = null;
					if(arc.cargo==-1) {
						if(arc.decision==-1 && (firstarcBranches[0][arc.head]==null || firstarcBranches[0][arc.head].get(GRB.DoubleAttr.RHS)> arc.value)) {
							con = model.addConstr(new GRBLinExpr(), GRB.LESS_EQUAL,arc.value, "Arc branch "+arc.cargo+" "+arc.head+" "+arc.value+" "+arc.decision);
							firstarcBranches[0][arc.head] = con;
							model.update();
						}
						else if(arc.decision==1 && (firstarcBranches[1][arc.head]==null || firstarcBranches[1][arc.head].get(GRB.DoubleAttr.RHS)< arc.value)) {
							con = model.addConstr(new GRBLinExpr(), GRB.GREATER_EQUAL,arc.value, "Arc branch "+arc.cargo+" "+arc.head+" "+arc.value+" "+arc.decision);
							firstarcBranches[1][arc.head] = con;
							model.update();
						}
					}
					else {
					
						if(arc.decision==-1) {
							if(arcBranches[0][arc.cargo][arc.head]==null) {  
							con = model.addConstr(arcExpr[0][arc.cargo][arc.head], GRB.LESS_EQUAL,arc.value, "Arc branch "+arc.cargo+" "+arc.head+" "+arc.value+" "+arc.decision);
							arcBranches[0][arc.cargo][arc.head] = con;
							if(branchingslack[0][arc.cargo][arc.head]==null) {
								branchingslack[0][arc.cargo][arc.head] = model.addVar(0,GRB.INFINITY,0,GRB.CONTINUOUS, "s");
								
							}
							obj.addTerm(-999999999, branchingslack[0][arc.cargo][arc.head]);
							model.update();
							model.chgCoeff(arcBranches[0][arc.cargo][arc.head], branchingslack[0][arc.cargo][arc.head], -1);
							}
							else if (arcBranches[0][arc.cargo][arc.head].get(GRB.DoubleAttr.RHS)> arc.value) {
								arcBranches[0][arc.cargo][arc.head].set(GRB.DoubleAttr.RHS, arc.value);
							}
//							System.out.println(arcBranches[0][arc.cargo][arc.head].get(GRB.DoubleAttr.RHS));
						}
						else if(arc.decision==1) {
							if(arcBranches[1][arc.cargo][arc.head]==null) { 
						
								con = model.addConstr(arcExpr[1][arc.cargo][arc.head], GRB.GREATER_EQUAL,arc.value, "Arc branch "+arc.cargo+" "+arc.head+" "+arc.value+" "+arc.decision);
								arcBranches[1][arc.cargo][arc.head] = con;
								if(branchingslack[1][arc.cargo][arc.head]==null) {
									branchingslack[1][arc.cargo][arc.head] = model.addVar(0,GRB.INFINITY,0,GRB.CONTINUOUS, "s");
									
								}
								obj.addTerm(-999999999, branchingslack[1][arc.cargo][arc.head]);
								model.update();
								model.chgCoeff(arcBranches[1][arc.cargo][arc.head], branchingslack[1][arc.cargo][arc.head], 1);
								
							}
							else if(arcBranches[1][arc.cargo][arc.head].get(GRB.DoubleAttr.RHS)< arc.value) {
								arcBranches[1][arc.cargo][arc.head].set(GRB.DoubleAttr.RHS, arc.value);
							}
							System.out.println(arcBranches[1][arc.cargo][arc.head].get(GRB.DoubleAttr.RHS));
						}
					}
					model.update();
				}
				else if(tm.getBranches().get(b) instanceof TwoArcBranch) {
					TwoArcBranch arc = (TwoArcBranch) tm.getBranches().get(b);
					System.out.println("Arcbranch "+arc.cargo+" "+arc.middle+" "+arc.head+" "+arc.decision+" "+arc.value);
					GRBConstr con = null;
					twoarcBranchExists=true;
					if(arc.decision==-1) {
						if(twoArcBranches[0][arc.cargo][arc.middle][arc.head]==null) {  
							con = model.addConstr(new GRBLinExpr(), GRB.LESS_EQUAL,arc.value, "Two Arc branch "+arc.cargo+" "+arc.middle+" "+arc.head+" "+arc.value+" "+arc.decision);
							twoArcBranches[0][arc.cargo][arc.middle][arc.head] = con;
						if(twobranchingslack[0][arc.cargo][arc.middle][arc.head]==null) {
							twobranchingslack[0][arc.cargo][arc.middle][arc.head] = model.addVar(0,GRB.INFINITY,0,GRB.CONTINUOUS, "s");
						}
						obj.addTerm(-999999999, twobranchingslack[0][arc.cargo][arc.middle][arc.head]);
						model.update();
						model.chgCoeff(twoArcBranches[0][arc.cargo][arc.middle][arc.head], twobranchingslack[0][arc.cargo][arc.middle][arc.head], -1);
						}
						else if (twoArcBranches[0][arc.cargo][arc.middle][arc.head].get(GRB.DoubleAttr.RHS)> arc.value) {
							twoArcBranches[0][arc.cargo][arc.middle][arc.head].set(GRB.DoubleAttr.RHS, arc.value);
						}
					}
					else if(arc.decision==1) {
						if(twoArcBranches[1][arc.cargo][arc.middle][arc.head]==null) {  
							con = model.addConstr(new GRBLinExpr(), GRB.GREATER_EQUAL,arc.value, "Two Arc branch "+arc.cargo+" "+arc.middle+" "+arc.head+" "+arc.value+" "+arc.decision);
							twoArcBranches[1][arc.cargo][arc.middle][arc.head] = con;
							System.out.println(arc.cargo);
							System.out.println(arc.middle);
							System.out.println(arc.head);
							if(twobranchingslack[1][arc.cargo][arc.middle][arc.head]==null) {
								twobranchingslack[1][arc.cargo][arc.middle][arc.head] = model.addVar(0,GRB.INFINITY,0,GRB.CONTINUOUS, "s");
							}
							obj.addTerm(-999999999, twobranchingslack[1][arc.cargo][arc.middle][arc.head]);
							model.update();
							model.chgCoeff(twoArcBranches[1][arc.cargo][arc.middle][arc.head], twobranchingslack[1][arc.cargo][arc.middle][arc.head], 1);
							}
							else if (twoArcBranches[1][arc.cargo][arc.middle][arc.head].get(GRB.DoubleAttr.RHS)< arc.value) {
								twoArcBranches[1][arc.cargo][arc.middle][arc.head].set(GRB.DoubleAttr.RHS, arc.value);
							}
					}
				}
			}
//			if(arcBranchExists) {
//				for(int i = 0; i < variables.size(); i++) {
//					int[][] coeff = new int[cargos.size()*2][cargos.size()*2+1];
//					int counter = 0;
//					Path p = pathList.get(allPaths.get(variables.get(i)));
//					if(p.portList.get(1).getId()<cargos.size()) {
//						if(firstarcBranches[0][p.portList.get(1).getId()]!=null) {
//							model.chgCoeff(firstarcBranches[0][p.portList.get(1).getId()], variables.get(i), 1);
//						}
//						if(firstarcBranches[1][p.portList.get(1).getId()]!=null) {
//							model.chgCoeff(firstarcBranches[1][p.portList.get(1).getId()], variables.get(i), 1);
//						}
//					}
//					
//					int previous = -1;
//					int current = p.getPortList().get(1).getId();
//					for(int j = 2; j < p.getPortList().size(); j++) {
//						previous = current;
//						current = p.getPortList().get(j).getId();
////						if(previous == arc.cargo && current == arc.head) {
//							coeff[previous][current]++;
//							
////						}
//					}
////					if(p.getPortList().size()==18) {
////						System.out.println(counter+" "+p.toString());
////					}
//					for(int j = 0; j < cargos.size()*2; j++) {
//						for(int k = 0; k < cargos.size()*2+1; k++) {
//							if(arcBranches[0][j][k]!=null) {
//								model.chgCoeff(arcBranches[0][j][k], variables.get(i),coeff[j][k]);
//							}
//							if(arcBranches[1][j][k]!=null) {
//								model.chgCoeff(arcBranches[1][j][k], variables.get(i),coeff[j][k]);
//							}
//						}
//					}
////					model.chgCoeff(con,variables.get(i), counter);
//					model.update();
//				}
//			}
			if(twoarcBranchExists) {
				for(int i = 0; i < variables.size(); i++) {
					int[][][] coeff = new int[cargos.size()*2][cargos.size()*2][cargos.size()*2+1];
					int counter = 0;
					Path p = pathList.get(allPaths.get(variables.get(i)));
					if(p.getPortList().size()>2) {
					int previous = p.getPortList().get(1).getId();
					int current = p.getPortList().get(2).getId();
					int next = 1;
					for(int j = 3; j < p.getPortList().size(); j++) {
						
						next = p.getPortList().get(j).getId();
//						if(previous == arc.cargo && current == arc.head) {
							coeff[previous][current][next]++;
							
//						}
						previous = current;
						current = next;
					}
//					if(p.getPortList().size()==18) {
//						System.out.println(counter+" "+p.toString());
//					}
					for(int j = 0; j < cargos.size()*2; j++) {
						for(int k = 0; k < cargos.size()*2; k++) {
							for(int l = 0; l < cargos.size()*2+1; l++) {
								if(twoArcBranches[0][j][k][l]!=null) {
									model.chgCoeff(twoArcBranches[0][j][k][l], variables.get(i),coeff[j][k][l]);
								}
								if(twoArcBranches[1][j][k][l]!=null) {
									model.chgCoeff(twoArcBranches[1][j][k][l], variables.get(i),coeff[j][k][l]);
								}
							}
						}
					}
				}
				}
			}
		//}
			model.update();
			this.nonOverFullGroupCuts3 = new GRBConstr[cargos.size()];
			for(int i = 0; i < cargos.size();i++) {
				if(z[i].get(GRB.DoubleAttr.LB)>=2 && tm.branchingCut1) {
					GRBConstr con =model.addConstr(new GRBLinExpr(), GRB.LESS_EQUAL, 0, "branching cut on cargo "+i);
					model.update();
//					con.setType(XPRB.L);
//					con.add(0);
//					con.setModCut(true);
					tm.branchCuts++;
					for(GRBVar var : variables) {
						Path p = pathList.get(allPaths.get(var));
						if(checkForNonOverFullGroup(i, p)) {
							model.chgCoeff(con, var, 1);
						}
					}
					this.nonOverFullGroupCuts3[i] = con;
//					for(int j = i+1; j < cargos.size();j++) {
//						if(z[i].getUB()<=1 && z[j].getUB()<=1) {
//							addOverFullGroupCut(i, j);
//						}
//						else if(z[i].getUB()<= 1 && z[j].getUB()>1) {
//							addOverFullGroupCut2(i, j);
//						}
//					}
					
				}
				
			}
			Date date2 = new Date();
			tm.setBoundsTime+= date2.getTime()-date1.getTime();
			return true;
	}
	
	private boolean checkFractionalFlow(double[][] arcflow) {
		for(int i = 0; i < cargos.size()*2; i++) {
			for(int j = 0; j < cargos.size()*2+1; j++) {
				if(arcflow[i][j]-Math.floor(arcflow[i][j])>tm.getZeroTol() && Math.ceil(arcflow[i][j])-arcflow[i][j]>tm.getZeroTol()) {
					return true;
				}
			}
			
		}
		return false;
	}
	
	public RouteBuilder getRoute() {
		return route;
	}
	
	public void printBounds(){
		try{
			for(int i = 0; i < cargos.size(); i++) {
				System.out.println("optional LB-bound: "+optionalVars[i].get(GRB.DoubleAttr.LB));
				System.out.println("optional UB-bound: "+optionalVars[i].get(GRB.DoubleAttr.UB));
				System.out.println("z LB-bound: "+z[i].get(GRB.DoubleAttr.LB));
				System.out.println("z UB-bound: "+z[i].get(GRB.DoubleAttr.UB));
			}
		}
		catch(Exception e) {
			e.printStackTrace();
			System.exit(0);
		}
	}
	
	public void printDuals() throws Exception{
//		try{
		System.out.println("ship dual: "+ships.get(0).getDual());
		double[] nonOverfullGroupDuals = route3.getGroupDuals();
			for(int i = 0; i < cargos.size(); i++) {
				System.out.println("Cargo "+i);
				System.out.println("Quantity "+cargos.get(i).getQuantity());
				System.out.println("dual: "+cargos.get(i).getDual());
				System.out.println("dual2: "+cargos.get(i).getDual2());
				System.out.println("cargo dual: "+cargos.get(i).getQuantityDual());
				System.out.println("group duals "+nonOverfullGroupDuals[i]);
//				if(nonOverFullGroupCuts3[i]!= null) {
//					System.out.println(nonOverFullGroupCuts3[i].get(GRB.DoubleAttr.Pi));
//				}
		}
//		catch(Exception e) {
//			e.printStackTrace();
//			System.exit(0);
//		}
	}
	
	public void findPath(BBNode node) {
//		System.out.println("cargo 6 quantity dual "+cargos.get(1).getQuantityDual());
		
		Path p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
	    
		
		int[] liste = {0, 6, 8, 2, 14, 10, 1, 9, 3, 11, 16};
		double[] 	temp =   {3.0, 3.0, 3.0, 3.0, 0.0, 0.0, 1.0, 0.0};
		p.cargoVolume = temp;
		for(int i = 0; i < liste.length; i++) {
			p.portList.add(ports.get(liste[i]));
		}
		try{
		comparePaths2(p, node);
		route3.recalcPath(p);
//		route.recalcPath(p);
		p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
		int[] liste2 = {4, 12, 6, 5, 13, 7, 14, 15, 16};
		double[] 	temp2 =   {0.0, 0.0, 0.0, 0.0, 3.0, 2.0, 1.0, 3.0};
		p.cargoVolume = temp2;
		for(int i = 0; i < liste2.length; i++) {
			p.portList.add(ports.get(liste2[i]));
		}
//		
//		
		comparePaths2(p, node);
		route3.recalcPath(p);
//		p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//		int[] liste3 = { 8, 24, 7, 1, 17, 15, 23, 5, 21, 31, 12, 28, 10, 11, 26, 27, 3, 19, 32};
//		double[] temp3 = {0.0, 1.0, 0.0, 6.0, 0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 3.0, 3.0, 1.0, 0.0, 0.0, 2.0};
////		temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
//		p.cargoVolume = temp3;
//		for(int i = 0; i < liste3.length; i++) {
//			p.portList.add(ports.get(liste3[i]));
//		}
//			
//			
//			comparePaths2(p, node);
//			route3.recalcPath(p);
//			p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//			double[] temp4 = {6.0, 0.0, 5.0, 0.0, 6.0, 0.0, 3.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 3.0, 0.0, 0.0};
//			int[] liste4 = {9, 25, 0, 16, 2, 18, 4, 20, 6, 10, 26, 22, 13, 29, 32};
////				temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
//				p.cargoVolume = temp4;
//			for(int i = 0; i < liste4.length; i++) {
//				p.portList.add(ports.get(liste4[i]));
//			}
//				
//				
//				comparePaths2(p, node);
//				route3.recalcPath(p);
//				p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//				int[] liste5 = {9, 39, 27, 0, 57, 30, 12, 42, 22, 6, 36, 8, 52, 38, 1, 31, 18, 48, 28, 58, 15, 45, 26, 24, 56, 54, 60};
//					double[] temp5 =   {3.0, 4.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 2.0, 3.0, 0.0, 0.0, 3.0, 0.0, 0.0, 6.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 4.0, 0.0, 1.0, 0.0, 3.0, 2.0, 2.0, 0.0};
//					p.cargoVolume = temp5;
//				for(int i = 0; i < liste5.length; i++) {
//					p.portList.add(ports.get(liste5[i]));
//				}
//					
//					
//				comparePaths2(p, node);
//				route3.recalcPath(p);
//				p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//				int[] liste6 = {17, 47, 6, 10, 36, 40, 5, 19, 35, 49, 16, 46, 13, 43, 26, 24, 56, 54, 60};
//				double[] temp6 =   {0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 5.0, 0.0, 0.0, 6.0, 5.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 3.0, 0.0, 0.0, 0.0};
//				p.cargoVolume = temp6;
//				for(int i = 0; i < liste6.length; i++) {
//					p.portList.add(ports.get(liste6[i]));
//				}
//					
//						
//				comparePaths2(p, node);
//				route3.recalcPath(p);
//				p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//							int[] liste7 = {23, 53, 29, 59, 20, 4, 50, 34, 5, 19, 35, 49, 16, 46, 13, 43, 7, 37, 11, 41, 14, 44, 2, 32, 60};
//							double[] temp7 =   {0.0, 0.0, 1.0, 0.0, 1.0, 5.0, 0.0, 5.0, 0.0, 0.0, 0.0, 5.0, 0.0, 5.0, 5.0, 0.0, 6.0, 0.0, 0.0, 1.0, 5.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0};
//							p.cargoVolume = temp7;
//				for(int i = 0; i < liste7.length; i++) {
//					p.portList.add(ports.get(liste7[i]));
//				}
//							
//							
//					comparePaths2(p, node);
//					route3.recalcPath(p);
//					p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//					int[] liste8 = {17, 47, 6, 10, 36, 40, 5, 19, 35, 49, 16, 46, 13, 43, 7, 37, 11, 41, 14, 44, 2, 32, 60};
////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
////					p.cargoVolume = temp;
//					double[] temp8 =   {0.0, 0.0, 1.0, 0.0, 0.0, 5.0, 0.0, 5.0, 0.0, 0.0, 6.0, 5.0, 0.0, 5.0, 5.0, 0.0, 6.0, 5.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//					p.cargoVolume = temp8;
//					for(int i = 0; i < liste8.length; i++) {
//						p.portList.add(ports.get(liste8[i]));
//					}
//					comparePaths2(p, node);
//					route3.recalcPath(p);
//					p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//					int[] liste9 = {17, 47, 6, 22, 36, 8, 52, 38, 1, 31, 18, 48, 28, 58, 15, 45, 26, 24, 56, 54, 60};
////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
////					p.cargoVolume = temp;
//					double[] temp9 =   {0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 5.0, 5.0, 0.0, 0.0, 0.0, 3.0, 0.0, 1.0, 0.0, 3.0, 0.0, 2.0, 0.0};
//					p.cargoVolume = temp9;
//					for(int i = 0; i < liste9.length; i++) {
//						p.portList.add(ports.get(liste9[i]));
//					}
//					comparePaths2(p, node);
//					route3.recalcPath(p);
//					p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//					int[] liste10 = {23, 53, 0, 12, 30, 42, 22, 6, 36, 8, 52, 38, 1, 31, 18, 48, 28, 58, 7, 37, 21, 25, 51, 55, 60};
//					double[] temp10 =   {3.0, 4.0, 0.0, 0.0, 0.0, 0.0, 3.0, 5.0, 2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 2.0, 3.0, 1.0, 0.0, 4.0, 0.0, 0.0, 2.0, 0.0};
//					p.cargoVolume = temp10;
////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
////					p.cargoVolume = temp;
//					for(int i = 0; i < liste10.length; i++) {
//						p.portList.add(ports.get(liste10[i]));
//					}
//					comparePaths2(p, node);
//					route3.recalcPath(p);
//					p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//					int[] liste11 = {9, 39, 27, 0, 57, 30, 12, 42, 22, 6, 36, 8, 52, 38, 1, 31, 18, 48, 28, 58, 7, 37, 21, 25, 51, 55, 60};
//					double[] temp11 =   {3.0, 4.0, 0.0, 0.0, 0.0, 0.0, 3.0, 5.0, 2.0, 3.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 2.0, 3.0, 0.0, 0.0, 4.0, 0.0, 2.0, 2.0, 0.0};
//					p.cargoVolume = temp11;
////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
////					p.cargoVolume = temp;
//					for(int i = 0; i < liste11.length; i++) {
//						p.portList.add(ports.get(liste11[i]));
//					}
//					comparePaths2(p, node);
//					route3.recalcPath(p);
//					p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//					int[] liste12 = { 9, 39, 27, 0, 57, 30, 12, 42, 22, 6, 36, 8, 52, 38, 1, 31, 18, 48, 28, 58, 15, 45, 11, 41, 14, 44, 2, 32, 60};
//					double[] temp12 =   {3.0, 4.0, 1.0, 0.0, 0.0, 0.0, 3.0, 0.0, 2.0, 3.0, 0.0, 5.0, 3.0, 0.0, 5.0, 6.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 0.0};
//					p.cargoVolume = temp12;
////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
////					p.cargoVolume = temp;
//					for(int i = 0; i < liste12.length; i++) {
//						p.portList.add(ports.get(liste12[i]));
//					}
//					comparePaths2(p, node);
//					route3.recalcPath(p);
//					p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//					int[] liste13 = {9, 39, 27, 0, 57, 30, 12, 42, 22, 6, 36, 8, 52, 38, 1, 3, 33, 31, 18, 48, 28, 58, 7, 37, 21, 25, 51, 55, 60};
//					double[] temp13 =   {3.0, 4.0, 0.0, 2.0, 0.0, 0.0, 3.0, 5.0, 2.0, 3.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 2.0, 3.0, 0.0, 0.0, 4.0, 0.0, 2.0, 2.0, 0.0};
//					p.cargoVolume = temp13;
////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
////					p.cargoVolume = temp;
//					for(int i = 0; i < liste13.length; i++) {
//						p.portList.add(ports.get(liste13[i]));
//					}
//					comparePaths2(p, node);
//					route3.recalcPath(p);
////					route3.recalcPath(p);
//					p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//					int[] liste14 = {23, 53, 29, 59, 20, 4, 50, 34, 3, 5, 33, 35, 15, 45, 26, 24, 56, 54, 60};
//					double[] temp14 =   {0.0, 0.0, 0.0, 4.0, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 1.0, 1.0, 0.0, 3.0, 0.0, 0.0, 4.0};
//					p.cargoVolume = temp14;
////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
////					p.cargoVolume = temp;
//					for(int i = 0; i < liste14.length; i++) {
//						p.portList.add(ports.get(liste14[i]));
//					}
//					comparePaths2(p, node);
//					route3.recalcPath(p);
//					route3.recalcPath(p);
//					p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//					int[] liste15 = {8, 15, 35, 28, 11, 31, 0, 10, 30, 20, 7, 4, 24, 4, 24, 27, 14, 34, 6, 26, 40};
////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
////					p.cargoVolume = temp;
//					for(int i = 0; i < liste15.length; i++) {
//						p.portList.add(ports.get(liste15[i]));
//					}
//					comparePaths2(p, node);
////					route3.recalcPath(p);
//					route3.recalcPath(p);
//					p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//					int[] liste16 = {15, 35, 8, 11, 28, 31, 0, 20, 7, 12, 32, 27, 12, 32, 14, 34, 14, 34, 6, 26, 40};
////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
////					p.cargoVolume = temp;
//					for(int i = 0; i < liste16.length; i++) {
//						p.portList.add(ports.get(liste16[i]));
//					}
//					comparePaths2(p, node);
////					route3.recalcPath(p);
//					route3.recalcPath(p);
//					p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//					int[] liste17 = {15, 35, 8, 11, 28, 31, 0, 20, 7, 12, 32, 27, 5, 25, 4, 24, 4, 24, 6, 14, 34, 26, 40};
////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
////					p.cargoVolume = temp;
//					for(int i = 0; i < liste17.length; i++) {
//						p.portList.add(ports.get(liste17[i]));
//					}
//					comparePaths2(p, node);
////					route3.recalcPath(p);
//					route3.recalcPath(p);
//					p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//					int[] liste18 = { 15, 35, 8, 11, 28, 31, 0, 10, 30, 10, 30, 10, 30, 10, 30, 10, 30, 20, 7, 12, 32, 27, 5, 25, 4, 24, 4, 24, 6, 14, 34, 26, 40};
////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
////					p.cargoVolume = temp;
//					for(int i = 0; i < liste18.length; i++) {
//						p.portList.add(ports.get(liste18[i]));
//					}
//					comparePaths2(p, node);
////					route3.recalcPath(p);
//					route3.recalcPath(p);
//					p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//					int[] liste19 = {8, 11, 28, 15, 35, 31, 13, 33, 2, 22, 17, 37, 17, 37, 18, 7, 38, 27, 12, 32, 12, 32, 14, 19, 34, 39, 1, 21, 40};
////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
////					p.cargoVolume = temp;
//					for(int i = 0; i < liste19.length; i++) {
//						p.portList.add(ports.get(liste19[i]));
//					}
//					comparePaths2(p, node);
//					route3.recalcPath(p);
////					route3.recalcPath(p);
////					int[] liste19 = {8, 11, 28, 15, 35, 31, 13, 33, 2, 22, 2, 22, 2, 22, 17, 37, 17, 37, 18, 7, 38, 27, 12, 32, 12, 32, 14, 19, 34, 39, 1, 21, 40};
//////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
//////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
//////					p.cargoVolume = temp;
////					for(int i = 0; i < liste13.length; i++) {
////						p.portList.add(ports.get(liste13[i]));
////					}
////					comparePaths2(p, node);
////					route3.recalcPath(p);
////					route3.recalcPath(p);
//					p = new Path(ships.get(0), ports.get(-1), 0, 0, 0, cargos.size());
//					int[] liste20 = {15, 35, 8, 11, 28, 31, 0, 10, 30, 20, 7, 5, 25, 27, 12, 32, 19, 4, 24, 4, 24, 39, 1, 21, 40};
////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
////					p.cargoVolume = temp;
//					for(int i = 0; i < liste20.length; i++) {
//						p.portList.add(ports.get(liste20[i]));
//					}
//					comparePaths2(p, node);
//					route3.recalcPath(p);
////					route3.recalcPath(p);
////					int[] liste13 = {8, 11, 28, 15, 35, 31, 13, 33, 2, 22, 2, 22, 2, 22, 17, 37, 17, 37, 18, 7, 38, 27, 12, 32, 12, 32, 14, 19, 34, 39, 1, 21, 40};
//////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
//////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
//////					p.cargoVolume = temp;
////					for(int i = 0; i < liste13.length; i++) {
////						p.portList.add(ports.get(liste13[i]));
////					}
////					comparePaths2(p, node);
////					route3.recalcPath(p);
////					route3.recalcPath(p);
////					int[] liste13 = {8, 11, 28, 15, 35, 31, 13, 33, 2, 22, 2, 22, 2, 22, 17, 37, 17, 37, 18, 7, 38, 27, 12, 32, 12, 32, 14, 19, 34, 39, 1, 21, 40};
//////					double[] temp2 = {4.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 4.0, 0.0, 0.0};
//////					temp =   {0.0, 2.0, 0.0, 6.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0};
//////					p.cargoVolume = temp;
////					for(int i = 0; i < liste13.length; i++) {
////						p.portList.add(ports.get(liste13[i]));
////					}
////					comparePaths2(p, node);
////					route3.recalcPath(p);
////		}
////		route.recalcPath(p);
		}
		catch(Exception e) {
			e.printStackTrace();
			System.exit(0);
		}
	}
	
	private void removeVariables() throws Exception{
		
		if(true) {
			return;
		}
		
		if(variables.size()>10000) {
			Collections.sort(variables, new VarComparator());
//			System.out.println("Size of Variables: "+variables.size());
//			System.out.println(variables.get(0).get(GRB.DoubleAttr.RC));
//			System.out.println(variables.get(5000).get(GRB.DoubleAttr.RC));
			//variables.retainAll(variables.subList(0, 3000));
			for(int i = variables.size()-1; i >= 3000;  i--) {
				model.remove(variables.get(i));
				obj.remove(variables.get(i));
				variables.remove(i);
			}
//			System.out.println(variables.get(0).get(GRB.DoubleAttr.RC));
//			System.out.println("Size of Variables: "+variables.size());
//			System.out.println(variables.get(2999).get(GRB.DoubleAttr.RC));
//			System.exit(0);
		}
	}
}
