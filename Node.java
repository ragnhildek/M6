import java.util.Vector;

public class Node {
	public int number; 
	public String type;
	public int location;
	public String locationName;
	public int weight;
	public int volume;
	public float earlyTimeWindow;
	public float lateTimeWindow;
	
	
	public Node(int number) {
		this.number = number;
	}
	
	public void getLocation(int location){
	
		switch(location) {
	case 1:
       locationName= " Fredrikstad";
       break;
   case 2:
       locationName= "R�ros";
       break;
   case 3:
       locationName= " Steinkjer";
       break;
   case 4:
       locationName= "Beitost�len";
       break;
   case 5:
       locationName= " Drammen";
       break;
   case 6:
       locationName= "T�nsberg";
       break;
   case 7:
       locationName= " Kongsvinger";
       break;
   case 8:
       locationName= "Elverum";
       break;
   case 9:
       locationName= " Alvdal";
       break;
   case 10:
       locationName= "Hjerkinn";
       break;
   case 11:
       locationName= " Meldal";
       break;
   case 12:
       locationName= "Mer�ker";
       break;
   case 13:
       locationName= " Hassdalen";
       break;
   case 14:
       locationName= "Krokstad�ra";
       break;
   case 15:
       locationName= "Sistranda";
       break;
   case 16:
       locationName= "Vinstra";
       break;
   case 17:
       locationName= " Kongsberg";
       break;
   case 18:
       locationName= "Nesbyen";
       break;
   case 19:
       locationName= " Trondheim";
       break;
   case 20:
       locationName= "Orkanger";
       break;
   case 21:
       locationName= " Tanem";
       break;
   case 22:
       locationName= "Meldal";
       break;
   case 23:
       locationName= " Vinje�ra";
       break;
   case 24:
       locationName= "Berk�k";
       break;
   case 25:
       locationName= " Oppdal";
       break;
   case 26:
       locationName= "Tynset";
       break;
   case 27:
       locationName= " Rennebu";
       break;
   case 28:
       locationName= "Domb�s";
       break;
   case 29:
       locationName= " Elverum";
       break;
   case 30:
       locationName= "�ndalsnes";
       break;
   case 31:
       locationName= " Lillehammer";
       break;
   case 32:
       locationName= "Hamar";
       break;
   case 33:
       locationName= " Oslo";
       break;
   case 34:
       locationName= "Fagernes";
       break;
   case 35:
       locationName= " Lillestr�m";
       break;
   case 36:
       locationName= "Haltdalen";
       break;
   case 37:
       locationName= " Stj�rdal";
       break;
   case 38:
       locationName= "Grimsbu";
       break;
   case 39:
       locationName= " Koppang";
       break;
   case 40:
       locationName= "Brandbu";
       break;
   case 41:
       locationName= " Raufoss";
       break;
   case 42:
       locationName= "Geiranger";
       break;
   case 43:
       locationName= " Flisa";
       break;
   case 44:
       locationName= "Sk�bu";
       break;
   case 45:
       locationName= " Fossbergom";
       break;
   case 46:
       locationName= "Molde";
       break;
   case 47:
       locationName= " Kristiansund";
       break;
   case 48:
       locationName= "Hammarvika";
       break;
   case 49:
       locationName= " Porsgrunn";
       break;
   case 50:
       locationName= "�lesund";
       break;
   case 51:
	   locationName = "EndDepot";
	   break;
		}
	}
	
	public static Node getCorrespondingNode(Node node, Vector<Node>nodes) throws NullPointerException {
		if (node.type == "Depot"){
	        throw new NullPointerException("The depot node does not have a corresponding node");
		}
		int num = node.number;
		for(int i = 0; i<nodes.size();i++){
			if(node.type=="PickupNode"){
				if (nodes.get(i).number== num+1){
				Node node1 = nodes.get(i); 
				return node1;
				}
			}
			else if (node.type == "DeliveryNode"){
				if (nodes.get(i).number== num-1){
				Node node1 = nodes.get(i); 
				return node1;
				}	
			}		
		}  
		return null;
	}
}