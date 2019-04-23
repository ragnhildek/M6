package Objects;

import java.util.Comparator;

public class NodeComparator2 implements Comparator<BBNode>{

	@Override
	public int compare(BBNode o1, BBNode o2) {
		if(o1.getParent()==null) {
			return 1;
		}
		else if(o2.getParent()==null) {
			return -1;
		}
		else if(o1.getParent().isSolved() && !o2.getParent().isSolved()) {
			return 1;
		}
		else if(o2.getParent().isSolved() && !o1.getParent().isSolved()) {
			return -1;
		}
		else{
			if(o1.getParent().getUpperbound()>o2.getParent().getUpperbound()) {
				return -1;
			}
			else if(o1.getParent().getUpperbound()<o2.getParent().getUpperbound()){
				return 1;
			}
		}
		return 0;	
	}
//	public int compare(BBNode o1, BBNode o2) {
//		if(o1.getNodeId()>o2.getNodeId()) {
//			return -1;
//		}
//		else {
//			return 1;
//		}
//	}

}
