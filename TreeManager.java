//package ColumnGenerator;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Date;
import java.util.Hashtable;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Vector;

//import Objects.Cargo;
//import Objects.CargoBranch;
//import Objects.Filewriter;
//import Objects.NodeComparator2;
//import Objects.Port;
//import Objects.Ship;
//import Objects.BBNode;
//import Objects.Branch;
//import Objects.NodeComparator;
//import Objects.Path;

public class TreeManager {
	private double zeroTol;
	
	private int testset;
	private int testinstant;
//	private Filewriter filewriter2;
	public boolean branchingCut1;
	public boolean useVolumeCuts;
	public int volumeCuts;
	public int branchCuts;
	public Vector<Double> subsetRowFlows;
	public int subsetRowCounter;
	public int knapsackcouter1;
	public int knapsackcouter2;
	public int knapsackcouter3;
	public int knapsackcouter4;
	public double knapsackcouter5;
	public int knapsackcouter6;
	public int knapsackcouter7;
	public int knapsackcouter8;
	public int knapsackcouter9;
	public int knapsackcouter10;
	public int knapsackcouter11;
	public int countSingleKnapsack;
	public int pathcounter1;
	public int pathcounter2;
	public int pathcounter3;
	public int pathcounter4;
	public int pathcounter5;
	public int pathcounter6;
	
	public int subproblemcounter1;
	public int subproblemcounter2;
	public int subproblemcounter3;
	public int subproblemcounter4;
	public int subproblemcounter5;
	public int subproblemcounter6;
	public int MaxIter;
	public Random random;
	public int dominanceCounter1;
	public int dominanceCounter2;
	public int dominanceCounter3;
	public int removeQuantityCOunter;
	public int removedVarCounter;
	
	public FileWriter filewriter;
	
	private ArrayList<Vehicle> vehicles;
//	private Hashtable<Integer,Port> ports;
	private ArrayList<Node> pickupNodes;
	private String testcase;
//	private XpressInterface3 xpi;
	private PriorityQueue<BBNode> nodes; 
//	private ArrayList<Branch> branches;
//	private ArrayList<CargoBranch> cargoBranches;
//	public ArrayList<CargoBranch> getCargoBranches() {
//		return cargoBranches;
//	}

	public void setCargoBranches(ArrayList<CargoBranch> cargoBranches) {
		this.cargoBranches = cargoBranches;
	}

	public int nodecount;
	public double UpperBound;
	public double LowerBound;
	private BBNode bestNode;
	public int totalTimeInMaster;
	public int totalTimeInSub;
//	public int totalTimeInCuts;
	
//	public int totalTimeIn6;
//	public int totalTimeIn4;
//	public int totalTimeInSub1;
//	public int totalTimeInSub2;
//	public int totalTimeInSub3;
//	public int totalTimeInSub4;
	public double timeOptSolFound;
	public boolean writeToFile;
//	public ArrayList<Branch> getBranches() {
//		return branches;
//	}

//	public void setBranches(ArrayList<Branch> branches) {
//		this.branches = branches;
//	}

	public TreeManager(String testcase, int testset, int testinstant) {
		try {
			filewriter = new Filewriter("Testset "+testset+" with correct k-path cuts.txt");
		//	filewriter2 = new Filewriter("Mandatory "+testset+".txt"); 
			//filewriter = new Filewriter("Cut tests full problem with volume-cuts.txt");
			//filewriter = new Filewriter("Testset "+testset+" with mandatory.txt");


		}
		catch(Exception e) {
			e.printStackTrace();
			System.exit(0);
		}
		this.branchingCut1 = true;
		this.useVolumeCuts = true;
		this.branchCuts = 0;
//		this.totalTimeIn4 = 0;
//		this.totalTimeIn6 = 0;
		this.subsetRowCounter=0;
		this.subsetRowFlows = new Vector<Double>();
		this.testset = testset;
		this.testinstant = testinstant;
		this.removeQuantityCOunter = 0;
		this.countSingleKnapsack=0;
		this.writeToFile = false;
		this.MaxIter = 100000;
		this.random = new Random(42);
		this.totalTimeInMaster = 0;
		this.totalTimeInSub = 0;
//		this.totalTimeInSub1 = 0;
//		this.totalTimeInSub2 = 0;
//		this.totalTimeInSub3 = 0;
//		this.totalTimeInSub4 = 0;
		this.removedVarCounter = 0;
//		this.totalTimeInCuts=0;
		this.vehicles = new ArrayList<Vehicle>();
//		this.ports = new Hashtable<Integer,Port>();
		this.pickupNodes = new ArrayList<Node>();
		this.testcase = testcase;
		this.zeroTol = 0.0001;
		this.nodes = new PriorityQueue<BBNode>(100, new BBNodeComparator());
		this.nodecount = 0;
		this.UpperBound = 999999999;
		this.LowerBound  = 0;
		this.branches = new ArrayList<Branch>();
		this.cargoBranches = new ArrayList<CargoBranch>();
		InputReader.readInput(testcase, ships, cargos, ports);
		for(int i = 0; i < cargos.size()*2; i++) {
			this.ports.get(i).setSortedPickup(ships.get(0), cargos, ports);
			this.ports.get(i).setSortedDelivery(ships.get(0), cargos, ports);
			if(i<cargos.size()) {
				System.out.println("optional "+i+":   "+cargos.get(i).getOptional());
			}
		}
//		Preprocess.generateMandatoryCargos(cargos, 0.25);
//		Preprocess.setMandatoryCargoes(cargos);

//		TwoPathCuts two = new TwoPathCuts(cargos, ports, this, ships);
//		two.findSubsets();
		//Preprocess.findSimultaneousCargoes(ships.get(0), cargos,ports,750);
		
//		pre.getTotalVolumeCuts();
//		Preprocess.transformCostMatrix(ships, cargos);
		this.xpi = new XpressInterface3(ships, ports, cargos, this.zeroTol, this);
		nodes.add(new BBNode(null, 0,0, 1000,0));
		
		nodecount++;
		nodes.peek().setBranchVariables(new Vector<Integer>());
		nodes.peek().setGeneratedVariables(new Vector<Integer>());
	}
	
	public ArrayList<Cargo> getCargos() {
		return cargos;
	}

	public void manageTree() {
		Vector<Integer> optionalVector = new Vector<Integer>();
		for(int i = 0; i < cargos.size(); i++) {
			optionalVector.add(cargos.get(i).getOptional());
		}
		//filewriter2.writeTestOutput("Test");
		filewriter2.writeTestOutput(optionalVector.toString());
 		filewriter2.flush();
		int counter = 0;
		Date date1 = new Date();
while(!nodes.isEmpty()) {
//	while(counter<1) {
			//System.out.println("Active Nodes: "+nodes.size());
			BBNode next = nodes.poll();
//			bestNode = next;
			solveNode(next);
			if(counter==0) {
				UpperBound = next.getObjectiveValue();
			}
			counter++;
			System.out.println("Solving node: "+counter+". Active nodes: "+nodes.size());
			System.out.println("Lower bound: "+LowerBound);
			System.out.println("objective value: "+next.getObjectiveValue()+" "+next.isSolved());
			System.out.println("Total time in Master: "+this.totalTimeInMaster);
			System.out.println("Total time in Sub "+this.totalTimeInSub);
			System.out.println("Total time in Sub part 1 "+this.totalTimeInSub1);
			System.out.println("Total time in Sub part 2 "+this.totalTimeInSub2);
			System.out.println("Total time in Sub part 3 "+this.totalTimeInSub3);
			System.out.println("Total time in Sub part 4 "+this.totalTimeInSub4);
			System.out.println("Total time in cuts "+this.totalTimeInCuts);
			System.out.println("Total number of cuts: "+xpi.getTwoPathCuts().subsets.size());
			//bestNode = next;
			if(counter%100==0) {
				System.gc();
			}
			Date date2 = new Date();
			double tid = (date2.getTime()-date1.getTime())/1000;
			if(tid > 36000) {
				String output;
				if(bestNode!=null) {
				output = testset+", "+testinstant+", "+ships.size()+","+cargos.size()+", Not Solved,"+UpperBound+","+bestNode.getIPsol()+
						","+timeOptSolFound+","+tid+","+(totalTimeInMaster/1000)+","+(totalTimeInSub/1000)+","+counter+","+xpi.pathList.size()+","+xpi.getTwoPathCuts().subsets.size()+","+this.volumeCuts+","+this.branchCuts+","+this.pathcounter6+","+xpi.checkForDublicatePaths();
				}
				else {
					output = testset+", "+testinstant+", "+ships.size()+","+cargos.size()+", Not Solved,"+UpperBound+","+0+
							","+timeOptSolFound+","+tid+","+(totalTimeInMaster/1000)+","+(totalTimeInSub/1000)+","+counter+","+xpi.pathList.size()+","+xpi.getTwoPathCuts().subsets.size()+","+this.volumeCuts+","+this.branchCuts+","+this.pathcounter6+","+xpi.checkForDublicatePaths();
					}
				filewriter.writeTestOutput(output);
				filewriter.flush();
				System.exit(0);
			}
		}
		


		Date date2 = new Date();
		double tid = (date2.getTime()-date1.getTime())/1000;
		if(bestNode==null) {
			String output = testset+", "+testinstant+", "+ships.size()+","+cargos.size()+", no feasible solution found";
			filewriter.writeTestOutput(output);
			filewriter.flush();
		}
		
		
		System.out.println("Execution time: "+tid);
		System.out.println("Solution in root node: "+UpperBound);
		//System.out.println("Objective Value: "+bestNode.getObjectiveValue());
		System.out.println("total time in Master: "+totalTimeInMaster);
		System.out.println("total time in Sub: "+totalTimeInSub);
		System.out.println("time until opt sol found: "+timeOptSolFound);
		System.out.println("NOde id "+bestNode.getNodeId());
		System.out.println("NOde solved "+bestNode.isSolved());
		for(Ship s : ships) {
			System.out.println(s.index);
		for(Path p : bestNode.getSolution()) {
			if(p.ship==s) {
				System.out.print(bestNode.getSolutionvars().get(p)+", ");
				System.out.println(p.toString());
			
//			for(int[] v : p.getDeliveryGroups(1)) {
//				for(int j = 0; j < cargos.size(); j++) {
//					System.out.print(v[j]+",");
//				}
//				System.out.println();
//			}
			}
		}
		}
		System.exit(0);
		String output = testset+", "+testinstant+", "+ships.size()+","+cargos.size()+",,"+UpperBound+","+bestNode.getIPsol()+
		","+timeOptSolFound+","+tid+","+(totalTimeInMaster/1000)+","+(totalTimeInSub/1000)+","+counter+","+xpi.pathList.size()+","+xpi.getTwoPathCuts().subsets.size()+","+this.volumeCuts+","+this.branchCuts+","+this.pathcounter6+","+xpi.checkForDublicatePaths();
//xpi.solveBenders(xpi.getXValues(next.getSolution(), next.getSolutionvars()));

		System.out.println(output);
		filewriter.writeTestOutput(output);
		filewriter.flush();
		
		System.out.println("Counter "+1+": "+this.pathcounter1);
		System.out.println("Counter "+2+": "+this.pathcounter2);
		System.out.println("Counter "+3+": "+this.pathcounter3);
		System.out.println("Counter "+4+": "+this.pathcounter4);
		System.out.println("Counter "+5+": "+this.pathcounter5);
		System.out.println("Counter "+6+": "+this.pathcounter6);
		System.out.println("subproblem counter 1 "+this.subproblemcounter1);
		System.out.println("subproblem counter 2 "+this.subproblemcounter2);
		System.out.println("subproblem counter 3 "+this.subproblemcounter3);
		System.out.println("subproblem counter 4 "+this.subproblemcounter4);
		System.out.println("subproblem counter 5 "+this.subproblemcounter5);
		System.out.println("subproblem counter 7 "+this.subproblemcounter6);
		System.out.println("Total times called "+this.dominanceCounter3);
		System.out.println("Dominance counter 1 "+this.dominanceCounter1);
		System.out.println("Dominance counter 2 "+this.dominanceCounter2);
		System.out.println("Number of same paths "+xpi.checkForDublicatePaths());
		System.out.println("NUmber of singleknapsack dominance "+this.countSingleKnapsack);
		System.out.println("number of quantity vars removed: "+this.removeQuantityCOunter);
//		System.out.println("Counter "+6+": "+this.knapsackcouter6);
//		System.out.println("Counter "+7+": "+this.knapsackcouter7);
//		System.out.println("Counter "+1+": "+this.knapsackcouter1);
//		System.out.println("Counter "+1+": "+this.knapsackcouter1);
//		
//		TwoPathCuts two = xpi.getTwoPathCuts();
		System.out.println("number Of Removed Vars: "+this.removedVarCounter);
		System.out.println("subsetRow counter "+this.subsetRowCounter);
		System.out.println(this.subsetRowFlows.toString());
		System.out.println("Total time in 6 "+this.totalTimeIn6);
		System.out.println("Total time in 4 "+this.totalTimeIn4);
//		for(int s = 0; s <ships.size(); s++) {
//			for(int i = 0; i < cargos.size(); i++) {
//				for(int j = i+1; j < cargos.size(); j++) {
//					if(xpi.shipCargoCuts[s][i][j]!= null){
//						if(xpi.shipCargoCuts[s][i][j].getDual()<0){
//						double calc = 0;
//						System.out.println("Constraint "+s+" "+i+" "+j);
//						for(Path path : bestNode.getSolution()) {
//							if(ships.indexOf(path.ship)==s && xpi.checkForSet(i, j, path)) {
//								System.out.println(path.toString2());
//								System.out.println(bestNode.getSolutionvars().get(path)*-1);
//								calc+=bestNode.getSolutionvars().get(path)*-1;
//							}
//							else if(ships.indexOf(path.ship)!=s){
//								if(path.getCargoesHandled()[i]>0 || path.getCargoesHandled()[j]>0) {
//									int ple = path.getCargoesHandled()[i] + path.getCargoesHandled()[j];
//									System.out.println(path.toString2());
//									System.out.println(bestNode.getSolutionvars().get(path)*ple);
//									calc+=bestNode.getSolutionvars().get(path)*ple;
//								}
//							}
//						}
//						System.out.println("Calc value "+calc);
//					}}
//				}
//			}
//		}
//		for(Vector v : two.subsets) {
//			System.out.print(v.toString()+"      ");
//			System.out.print(two.RHS.get(v)+"   ");
//			for(int i = 0; i < cargos.size(); i++) {
//				if(v.contains(i) || v.contains(i+cargos.size())) {
//					System.out.print(two.getYcoefficient(i, two.RHS.get(v))+" ");
//				}
//				else {
//					System.out.print("0 ");
//				}
//			}
//			System.out.println(two.getRightHandSide(v));
//		}
	}
	
	public void solveNode(BBNode node) {
//		for(int b : node.getBranches()) {
//			//System.out.println(" Branching on ship "+branches.get(b).getShip()+" and cargo "+branches.get(b).getCargo()+" decision "+branches.get(b).getDecision());
//			
//		}
		
		BBNode[] newnodes = xpi.dynamicGen(node);
		if(newnodes == null) {
			
			if(node.getObjectiveValue()>LowerBound) {
				timeOptSolFound = (this.totalTimeInMaster+this.totalTimeInSub)/1000;
				LowerBound = node.getIPsol();
				bestNode = node;
				ArrayList<BBNode> remove = new ArrayList<BBNode>();
				while(!nodes.isEmpty())  {
					BBNode n = nodes.poll();
//					if(n.getParent().getObjectiveValue()>= LowerBound || !n.getParent().isSolved())  {
					if(n.getParent().getUpperbound()>= LowerBound || !n.getParent().isSolved())  {
						remove.add(n);
					}
				}
				nodes = new PriorityQueue<BBNode>(100, new BBNodeComparator());
				for(BBNode n : remove) {
					nodes.add(n);
				}
			}
			if((!node.isSolved() || node.getObjectiveValue()!=node.getUpperbound()) && node.getObjectiveValue()>0) {
				if((totalTimeInMaster/1000)+(totalTimeInSub/1000)>36000) {
					String output = testset+", "+testinstant+", "+ships.size()+","+cargos.size()+", not solved";
					filewriter.writeTestOutput(output);
					filewriter.flush();
					System.exit(0);
				}
				solveNode(node);
			}
		}
		//else if(!node.isSolved() || node.getObjectiveValue()>=LowerBound) {
		else {	
		
			nodes.add(newnodes[0]);
			nodes.add(newnodes[1]);
		}
//		if(node.getNodeId()==118) {
//			for(Ship s : ships) {
//			for(Path p : node.getSolution()) {
//				if(p.ship==s) {
//					System.out.print(node.getSolutionvars().get(p)+", ");
//					System.out.println(p.toString());
//				
//			
//				}
//			}
//		}
//		//System.exit(0);
//		}
	}
	
	public void addNode(BBNode node) {
		nodes.add(node);
	}
	public void updateNodeCount() {
		nodecount++;
	}
	
	public int addBranch(Branch b) {
		branches.add(b);
		return branches.size()-1;
	}
	public double getZeroTol() {
		return zeroTol;
	}

	public void setZeroTol(double zeroTol) {
		this.zeroTol = zeroTol;
	}
	
	public int addCargoBranch(CargoBranch c) {
		cargoBranches.add(c);
		return cargoBranches.size()-1;
	}
}
