import com.gurobi.gurobi.*;

public class GurobiTest {
    public static void main(String[] args) {
        try {
            GRBEnv env = new GRBEnv("test.log");
            System.out.println("Gurobi is working!");
            env.dispose();
        } catch (GRBException e) {
            System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
        }
    }
}