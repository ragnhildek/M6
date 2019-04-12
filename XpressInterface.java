
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.Hashtable;
import java.util.Vector;

import com.dashoptimization.XOctrl;
import com.dashoptimization.XPRB;
import com.dashoptimization.XPRBbasis;
import com.dashoptimization.XPRBctr;
import com.dashoptimization.XPRBexpr;
import com.dashoptimization.XPRBprob;
import com.dashoptimization.XPRBvar;
import com.dashoptimization.XPRS;
import com.dashoptimization.XPRSprob;
import java.util.Date;




public class XpressInterface {
	private XPRB xprb = new XPRB();
	private XPRS xprs = new XPRS();
	private XPRBprob problem;
	
	private Hashtable<Integer, Label> pathList;
	
	//Variables
	private ArrayList<XPRBvar> variables;
	private XPRBvar[][] deltavars;
	
	//Constraints
	private XPRBctr[] assignmentCons;
	private XPRBctr[][] shipSailingCons;
	private XPRBctr[] DepotCons;
	private XPRBctr[][] charterOutCons;
	
	//objective
	private XPRBexpr objective;
	
	private double totalTimeInSub;
	private double totalTimeInMaster;
	
	public ArrayList<Vessel> vessels;
	public ArrayList<CustomerShip> customers;
	public InstanceData inputdata;
	public PathBuilder builder;
	private Filewriter filewriter;
	
	public XpressInterface(ArrayList<Vessel> vessels, ArrayList<CustomerShip> customers, InstanceData inputdata) {
		this.vessels = vessels;
		this.customers = customers;
		this.inputdata = inputdata;
		this.builder = new PathBuilder(vessels, customers, inputdata);
//		builder.BuildPaths(vessels.get(1),15);
//		System.exit(0);
		this.pathList = new Hashtable<Integer, Label>();
		this.filewriter = new Filewriter("output.txt");
		buildProblem();
		pregeneratePaths();
	}
	
	private void buildProblem() {
		
		this.problem = xprb.newProb("problem 1");
		problem.setCutMode(1);
		//this.problem.setXPRSdblControl(XPRS.OPTIMALITYTOL, 0.00000000001);
//		this.problem.setXPRSintControl(XOctrl.XPRS_PRESOLVE, 0);
		this.problem.setXPRSintControl(XOctrl.XPRS_OUTPUTLOG, 1);
		this.problem.setXPRSintControl(XOctrl.XPRS_MIPLOG, -100);
		this.problem.setXPRSintControl(XOctrl.XPRS_CUTSTRATEGY,0);
//		this.problem.setXPRSintControl(XOctrl.XPRS_MAXTIME,-1000);
//		this.problem.setXPRSintControl(XOctrl.XPRS_HEURSTRATEGY, 3);
//		this.problem.getXPRSprob().setIntControl(XPRS.HEURSTRATEGY, 3);
//		this.problem.getXPRSprob().setIntControl(XPRS.HEURSEARCHTREESELECT, 1100);
//		this.problem.setXPRSintControl(XPRS.THREADS, 1);
//		this.problem.getXPRSprob().setIntControl(XPRS.THREADS, 1);
//		this.problem.setMsgLevel(4);
//		this.problem.setDictionarySize(XPRB.DICT_NAMES, 0);
//		this.problem.setColOrder(1);
		this.objective = new XPRBexpr();
		this.variables = new ArrayList<XPRBvar>();
		this.deltavars = new XPRBvar[inputdata.nrTimePeriods/24][vessels.size()];
		
		
		//Constraints
		
		this.assignmentCons = new XPRBctr[customers.size()];
		this.shipSailingCons = new XPRBctr[inputdata.nrTimePeriods+1][vessels.size()];
		this.DepotCons = new XPRBctr[inputdata.nrTimePeriods+1];
		this.charterOutCons = new XPRBctr[inputdata.nrTimePeriods/24][vessels.size()];
//		
		for(int i = 0; i < customers.size(); i++) {
			assignmentCons[i] = problem.newCtr("con"+i);
			assignmentCons[i].setType(XPRB.E);
			assignmentCons[i].add(1);
		}
		for(int t = 0; t <=inputdata.nrTimePeriods; t++) {
			for(int i = 0; i < vessels.size(); i++) {
				this.shipSailingCons[t][i] = problem.newCtr("shipsailing con");
				this.shipSailingCons[t][i].setType(XPRB.L);
				this.shipSailingCons[t][i].add(1);
			}
		}
		for(int t = 0; t <=inputdata.nrTimePeriods; t++) {
			
				this.DepotCons[t] = problem.newCtr("shipsailing con");
				this.DepotCons[t].setType(XPRB.L);
				this.DepotCons[t].add(1);
			
		}
//		System.out.println("value: "+(inputdata.nrTimePeriods));
//		System.out.println("value: "+(inputdata.nrTimePeriods/24));
//		System.exit(0);
		for(int t = 0; t <(inputdata.nrTimePeriods/24); t++) {
			for(int i = 0; i < vessels.size(); i++) {
				System.out.println("creating charter out cons "+t+" "+i);
				
				this.deltavars[t][i] = problem.newVar("delta "+t+" "+i, XPRB.BV);
				this.objective.addTerm(this.deltavars[t][i], vessels.get(i).fixedCost);
				
				this.charterOutCons[t][i] = problem.newCtr("charter out con");
				this.charterOutCons[t][i].setType(XPRB.L);
				this.charterOutCons[t][i].add(0);
				this.charterOutCons[t][i].addTerm(this.deltavars[t][i],-24);
			}
		}
		
	}
	
	private void pregeneratePaths() {
		
//		int c = 0;
		Date time1 = new Date();
		for(Vessel v: vessels) {
			for(int t = v.startTime; t < inputdata.nrTimePeriods; t++) {
				ArrayList<Label> labels = builder.BuildPaths(v, t);
				for(Label l : labels) {
					addLabel(l);
				}
//				System.exit(0);
			}
			
		}
		Date time2 = new Date();
		totalTimeInSub = (time2.getTime()-time1.getTime())/1000.00;
		solveProblem();
	}
	
	private void addLabel(Label l) {
//		System.out.println(l.toString());
//		if(l.vessel.number==1 && l.VisitedNodes.size()>1) {
//			return;
//		}
//		if(checkForPath(path)) {
//		System.out.println("adding label: " + l.toString());
		XPRBvar var = problem.newVar("x "+variables.size(), XPRB.BV, 0, XPRB.INFINITY);
	
		variables.add(var);
		pathList.put(variables.size()-1, l);
		
		for(int i : l.VisitedNodes) {
			this.assignmentCons[i].addTerm(var,1);
		}
		for(int t = l.startTime; t <=l.time; t++) {
			this.shipSailingCons[t][l.vessel.number].addTerm(var,1);
		}
		for(int t = 0; t <inputdata.depotOperatingTime; t++) {
			this.DepotCons[t+l.startTime].addTerm(var,1);
		}
		for(int t = l.startTime; t <=l.time; t++) {
//				System.out.println(t/24);
				this.charterOutCons[t/24][l.vessel.number].addTerm(var,1);
		
		}
		
		this.objective.addTerm(var, l.cost);
//		}
	}
	
	
	
	
	private void solveProblem() {
		Date d1 = new Date();
		problem.setObj(this.objective);
//		problem.print();
		problem.setSense(XPRB.MINIM);
		problem.sync(XPRB.XPRS_PROB);
		
		problem.solve("l");
		XPRBbasis basis = problem.saveBasis();
		
//		problem.mipOptimize();
		Date d2 = new Date();
		totalTimeInMaster=(d2.getTime()-d1.getTime())/1000.00;
		printSolution();
	}
	
	private void printSolution() {
		for(int i = 0; i < variables.size(); i++) {
			if(variables.get(i).getSol()>0.9) {
				System.out.println(variables.get(i).getSol()+" "+pathList.get(i).toString());
			}
		}
		for(int t = 0; t <(inputdata.nrTimePeriods/24); t++) {
			for(int i = 0; i < vessels.size(); i++) {
				if(this.deltavars[t][i].getSol()>0.9) {
					System.out.println("solution delta "+t+" "+i+": "+this.deltavars[t][i].getSol());
				}
				
			}
		}

		XPRSprob p = problem.getXPRSprob();
		//int ple = p.getIntControl(XPRS.CPUTIME);
		//double ple = p.getDblControl(XPRS.CPUTIME);
//		
		//System.out.println(ple);
		int ple2 = p.getIntAttrib(XPRS.NODES);
		int ple = p.getIntControl(XPRS.THREADS);
		System.out.println(ple2);
		String output = inputdata.instanceName+", ";
		
		System.out.println(output);
		output+= problem.getObjVal()+", ";
		System.out.println(output);	
		output+= +totalTimeInSub+", ";
		System.out.println(output);
		output+=totalTimeInMaster+", "; 
		System.out.println(output);
		output+=pathList.size()+","; 
		System.out.println(output);
		output+= ple2+", "; 
		System.out.println(output);
		output+=ple+", "; 
		System.out.println(output);
		output+=problem.getMIPStat()+", ";
		System.out.println(output);
		
		problem.solve("l");
		output+= problem.getObjVal();
		System.out.println(output);	
		filewriter.writeTestOutput(output);
		filewriter.flush();
		
	}
	
} 
