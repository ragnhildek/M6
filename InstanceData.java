import java.util.ArrayList;


public class InstanceData {
	public String instanceName;
	public float fuelPrice;
	public float fuelConsumptionEmptyTruckPerKm;
	public float fuelConsumptionPerTonKm;
	public int laborCostperHour;
	public float otherDistanceDependentCostsPerKm;
	public int otherTimeDependentCostsPerKm;
	public float timeTonService;
	public int revenue;
	public ArrayList<Integer> nodesDepot;
	public ArrayList<Integer> nodes;
	public int volumeCap;
	public int weightCap;
	public float[][] times;
	public float[][] distances;
	public int numberOfCities;
	public int numberOfVehicles; 

	
	public InstanceData(String datafile) {
		this.instanceName = datafile;
	}
	
	public float getDistance (Node i, Node j){
	int iLocation = i.location;
	int jLocation = j.location;
	return distances[iLocation-1][jLocation-1];
	//System.out.println(distance);
	}
	
	public float getTime (Node i, Node j){
	int iTime = i.location;
	int jTime = j.location;
//	System.out.println(iTime);
//	System.out.println(jTime);
	return times[iTime-1][jTime-1];
	//System.out.println(time);
	}
}