package Objects;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Vector;

import com.dashoptimization.XPRBbasis;
import com.dashoptimization.XPRBvar;

import Objects.Path;

public class BBNode {
	private BBNode parent;
	private int depth;
	private BBNode leftChild;
	private BBNode rightChild;
	private double objectiveValue;
	private int nodeId;
	private Vector<Integer> branchVariables;
	private Vector<Integer> cargoBranches;
	private Vector<Integer> branches;
	private XPRBbasis basis;
	private XPRBbasis[] childBasis;
	private int numberOfCargoesBranchUpper;
	private int numberOfCargoesBranchLower;
	private double upperbound;
	private double IPsol;
	
	public double getIPsol() {
		return IPsol;
	}
	public void setIPsol(double psol) {
		IPsol = psol;
	}
	public double getUpperbound() {
		return upperbound;
	}
	public void setUpperbound(double upperbound) {
		this.upperbound = upperbound;
	}
	public void setNumberOfCargoesBranchUpper(int number) {
		this.numberOfCargoesBranchUpper = number;
	}
	public void setNumberOfCargoesBranchLower(int number) {
		this.numberOfCargoesBranchLower = number;
	}
	
	public int getNumberOfCargoesBranchUpper() {
		return numberOfCargoesBranchUpper;
	}
	public int getNumberOfCargoesBranchLower() {
		return numberOfCargoesBranchLower;
	}
	
	public XPRBbasis[] getChildBasis() {
		return childBasis;
	}

	public void setChildBasis(XPRBbasis[] childBasis) {
		this.childBasis = childBasis;
	}

	public XPRBbasis getBasis() {
		return basis;
	}

	public void setBasis(XPRBbasis basis) {
		this.basis = basis;
	}

	public Vector<Integer> getBranches() {
		return branches;
	}

	public void setCargoBranches(Vector<Integer> cargoBranches) {
		this.cargoBranches = cargoBranches;
	}

	private Vector<Integer> generatedVariables;
	private ArrayList<Path> solution;
	private Hashtable<Path, Double> solutionvars;
	public Hashtable<Path, Double> getSolutionvars() {
		return solutionvars;
	}

	public void setSolutionvars(Hashtable<Path, Double> solutionvars) {
		this.solutionvars = solutionvars;
	}

	private boolean solved;
	
	
	
	public boolean isSolved() {
		return solved;
	}

	public void setSolved(boolean solved) {
		this.solved = solved;
	}

	public ArrayList<Path> getSolution() {
		return solution;
	}

	public void setSolution(ArrayList<Path> solution) {
		this.solution = solution;
	}

	public Vector<Integer> getGeneratedVariables() {
		return generatedVariables;
	}

	public void setGeneratedVariables(Vector<Integer> generatedVariables) {
		this.generatedVariables = generatedVariables;
	}

	public BBNode(BBNode parent, int depth, int nodeId, int upper, int lower) {
		this.parent = parent;
		this.depth = depth;
		this.nodeId = nodeId;
		this.cargoBranches = new Vector<Integer>();
		this.branches = new Vector<Integer>();
		this.numberOfCargoesBranchUpper = upper;
		this.numberOfCargoesBranchLower = lower;

	}
	
	public void setObjectiveValue(double objVal) {
		this.objectiveValue = objVal;
	}
	
	public BBNode getParent() {
		return this.parent;
	}
	
	public void setBranchVariables(Vector<Integer> branchVariables) {
		this.branchVariables = branchVariables;
	}
	
	public Vector<Integer> getCargoBranches() {
		return cargoBranches;
	}

	public void setBranches(Vector<Integer> branches) {
		this.branches = branches;
	}

	public Vector<Integer> getBranchVariables() {
		return this.branchVariables;
	}
	

	public int getDepth() {
		return depth;
	}

	public void setDepth(int depth) {
		this.depth = depth;
	}

	public BBNode getLeftChild() {
		return leftChild;
	}

	public void setLeftChild(BBNode leftChild) {
		this.leftChild = leftChild;
	}

	public BBNode getRightChild() {
		return rightChild;
	}

	public void setRightChild(BBNode rightChild) {
		this.rightChild = rightChild;
	}

	public int getNodeId() {
		return nodeId;
	}

	public void setNodeId(int nodeId) {
		this.nodeId = nodeId;
	}

	public double getObjectiveValue() {
		return objectiveValue;
	}

	public void setParent(BBNode parent) {
		this.parent = parent;
	}
	
}
