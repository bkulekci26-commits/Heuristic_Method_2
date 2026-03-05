import java.io.*;
import java.util.*;

public class InstanceReader {

    /**
     * Reads instance with a specified sync window W.
     */
    public static Instance read(String filePath, double syncWindow) throws IOException {
        String name = "";
        int maxVehicles = 0;
        double maxCapacity = 0, maxTime = 0;
        double depotX = 0, depotY = 0;
        List<Node> nodes = new ArrayList<>();

        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            boolean readingCustomers = false;
            int nodeId = 1;

            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty()) continue;

                if (line.startsWith("NAME")) {
                    name = line.split("\\s+", 2)[1].trim();

                } else if (line.startsWith("MAXVEHICLES")) {
                    maxVehicles = Integer.parseInt(line.split("\\s+")[1].trim());

                } else if (line.startsWith("MAXCAPACITY")) {
                    maxCapacity = Double.parseDouble(line.split("\\s+")[1].trim());

                } else if (line.startsWith("MAXTIME")) {
                    maxTime = Double.parseDouble(line.split("\\s+")[1].trim());

                } else if (line.startsWith("DEPOT")) {
                    String[] parts = line.split("\\s+");
                    depotX = Double.parseDouble(parts[1]);
                    depotY = Double.parseDouble(parts[2]);
                    nodes.add(new Node(0, depotX, depotY, 0, 0, true));

                } else if (line.startsWith("CUSTOMERDATA")) {
                    readingCustomers = true;

                } else if (readingCustomers) {
                    String[] parts = line.trim().split("\\s+");
                    if (parts.length < 5) continue;

                    double x       = Double.parseDouble(parts[0]);
                    double y       = Double.parseDouble(parts[1]);
                    double demand  = Double.parseDouble(parts[2]);
                    // parts[3] = service time (not used in current model)
                    double profit  = Double.parseDouble(parts[4]);

                    nodes.add(new Node(nodeId++, x, y, demand, profit, false));
                }
            }
        }

        return new Instance(name, nodes, 0, maxVehicles, maxCapacity, maxTime, syncWindow);
    }

    /**
     * Reads instance with default sync window = Tmax (no sync restriction initially).
     */
    public static Instance read(String filePath) throws IOException {
        // First pass to get Tmax, then create with W = Tmax
        return read(filePath, Double.MAX_VALUE); // will be overridden once we know Tmax
    }
}