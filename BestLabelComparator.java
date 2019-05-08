//import java.util.Vector;
import java.util.Comparator;

public class BestLabelComparator implements Comparator<Label>{

	// Sorting the list of unprocessed labels such that the label with highest reduced cost is selected first
    public int compare(Label label1, Label label2)
    {
        if (label1.reducedCost > label2.reducedCost)
        {
            return -1;
        }
        else if (label1.reducedCost < label2.reducedCost)
        {
            return 1;
        }
        return 0;
    }	


}
