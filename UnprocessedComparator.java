//import java.util.Vector;
import java.util.Comparator;

public class UnprocessedComparator implements Comparator<Label> {
	// Sorting the list of unprocessed labels such that the label with smallest time is selected first
    public int compare(Label label1, Label label2)
    {
        if (label1.time < label2.time)
        {
            return -1;
        }
        else if (label1.time > label2.time)
        {
            return 1;
        }
        return 0;
    }	
}
