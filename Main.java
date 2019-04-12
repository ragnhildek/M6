import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Vector;


public class Main {

	public static void main(String[] args) throws Exception {
		long startTime = System.nanoTime();
		
		String datafile = "10P5R2"
				+ "V_java.txt";
		
		File file = new File ("8P5R1V_java_results.txt");
		
		if (!file.exists()) {
			try { file.createNewFile(); 
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		
		PrintWriter pw = new PrintWriter(file);	
	//	Vector<Node> nodes = new Vector<Node>();
		Vector<Node> nodesWithoutDepot = new Vector<Node>();
		Vector<Node> pickupNodes = new Vector<Node>();
		Vector<Node> deliveryNodes = new Vector<Node>();
		Vector<Node> startDepots = new Vector<Node>();
		Vector<Vehicle> vehicles = new Vector<Vehicle>();
	//	Vector<Route> routes = new Vector<Route>();
	//	Vehicle vehicle = new Vehicle();
		Vector<Float> dualVisitedPickupsCon = new Vector<Float>();  
		Vector<Float> dualOneVisitCon = new Vector<Float>();
		//Vector<Float> dualVisitedPickupsCon;
		//Vector<Float> dualOneVisitCon;
		
		
		
		
		InstanceData inputdata = new InstanceData(datafile);
		//PathBuilder builder = new PathBuilder(pickupNodes, deliveryNodes, nodes, depot,inputdata, pw, routes, vehicles);

		InputReader.inputReader(datafile, inputdata, nodesWithoutDepot, pickupNodes, deliveryNodes, startDepots, vehicles);
	//	System.out.println(inputdata.times[2][18]);

		GurobiInterface solver = new GurobiInterface(inputdata, deliveryNodes,  pickupNodes, nodesWithoutDepot, vehicles, dualVisitedPickupsCon, dualOneVisitCon, pw);
		solver.solveProblem();
		//PathBuilder builder;
		
		//builder.BuildPaths(vehicle);
		
		//time
		long endTime = System.nanoTime();
		System.out.println("Took "+(endTime - startTime)/1000000 + " milli seconds"); 
		pw.println ("Took "+(endTime - startTime)/1000000 + " milli seconds");
		
		pw.close();
	}
}
