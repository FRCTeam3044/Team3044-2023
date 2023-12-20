package frc.packages.pathfinding;

import java.net.URI;
import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.util.ArrayList;
import java.util.Base64;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.packages.pathfinding.Structures.Node;
import frc.packages.pathfinding.Structures.Path;
import frc.robot.commands.Drive.StartPathfindCommand.PathfindSnapMode;

/**
 * This example demonstrates how to create a websocket connection to a server. Only the most
 * important callbacks are overloaded.
 */
public class PathfindingWS extends WebSocketClient {

    public boolean hasRecievedPath = false;
    public boolean error = false;
    public Path path;

    private final Pathfinder localPathfinder;
    private boolean usingWebsocket = false;

    public PathfindingWS(URI serverURI, Pathfinder m_pathfinder) {
        super(serverURI);
        localPathfinder = m_pathfinder;
    }

    @Override
    public void onOpen(ServerHandshake handshakedata) {
        System.out.println("opened connection");
        usingWebsocket = true;
        // if you plan to refuse connection based on ip or httpfields overload: onWebsocketHandshakeReceivedAsClient
    }

    @Override
    public void onMessage(String message) {
        if (message.startsWith("ipe")) {
            System.out.println("Impossible Path Exception");
            error = true;
        } else if (message.startsWith("err")) {
            System.out.println("Pathfinding Exception: " + message.substring(4));
            error = true;
        } else if (message.startsWith("path")) {
            String pathRaw = message.split(" ")[1];
            double[] pathDoubleArr = decodeLocation(pathRaw);
            path = Path.fromDoubleArray(pathDoubleArr);
            hasRecievedPath = true;
            path.sendToSmartDashboard();
        } else {
            System.out.println("received unkown message: " + message);
        }
    }

    @Override
    public void onClose(int code, String reason, boolean remote) {
        usingWebsocket = false;
        // The close codes are documented in class org.java_websocket.framing.CloseFrame
        System.out.println("WS Connection closed" + " Code: " + code + " Reason: " + reason);
    }

    @Override
    public void onError(Exception ex) {
        usingWebsocket = false;
        if (ex instanceof java.net.ConnectException) {
            System.out.println("Unable to connect to Potato Pathfinding Server");
        } else {
            ex.printStackTrace();
        }

        // if the error is fatal then onClose will be called additionally
    }

    public void sendPathfindingRequest(Node start, Node end, PathfindSnapMode snapMode, ArrayList<Node> additionalVerticies) {
        if (!usingWebsocket) {
            localPathfind(start, end, snapMode, additionalVerticies != null ? additionalVerticies : new ArrayList<>());
            return;
        }
        error = false;
        hasRecievedPath = false;
        path = null;
        try {
            double[] startArray = {start.x, start.y};
            double[] endArray = {end.x, end.y};
            String startString = encodeLocation(startArray);
            String endString = encodeLocation(endArray);

            String dynamicVerticesString;
            if (additionalVerticies != null) {
                ArrayList<Double> dynamicVerticies = new ArrayList<>();
                for (Node n : additionalVerticies) {
                    dynamicVerticies.add(n.x);
                    dynamicVerticies.add(n.y);
                }
                double[] dynamicVerticesArray = new double[dynamicVerticies.size()];
                for (int i = 0; i < dynamicVerticesArray.length; i++) {
                    dynamicVerticesArray[i] = dynamicVerticies.get(i);
                }
                dynamicVerticesString = encodeLocation(dynamicVerticesArray);
            } else {
                dynamicVerticesString = "null";
            }


            String message = "sp " + startString + " " + endString + " " + (DriverStation.getAlliance() == Alliance.Red ? "t" : "f") + " " + snapMode.ordinal() + " " + dynamicVerticesString;

            send(message);
        } catch (Exception e) {
            e.printStackTrace();
            error = true;
        }

    }

    public Path getPath() {
        Path temp = path;
        path = null;
        hasRecievedPath = false;
        error = false;
        return temp;
    }

    private static String encodeLocation(double[] doubleArray) {
        return Base64.getEncoder().encodeToString(doubleToByteArray(doubleArray));
    }

    private static byte[] doubleToByteArray(double[] doubleArray) {
        ByteBuffer buf = ByteBuffer.allocate(Double.SIZE / Byte.SIZE * doubleArray.length);
        buf.asDoubleBuffer().put(doubleArray);
        return buf.array();
    }

    private static double[] decodeLocation(String base64Encoded) {
        return byteToDoubleArray(Base64.getDecoder().decode(base64Encoded));
    }

    private static double[] byteToDoubleArray(byte[] bytes) {
        DoubleBuffer buf = ByteBuffer.wrap(bytes).asDoubleBuffer();
        double[] doubleArray = new double[buf.limit()];
        buf.get(doubleArray);
        return doubleArray;
    }


    public void localPathfind(Node start, Node end, PathfindSnapMode snapMode, ArrayList<Node> additionalVerticies) {
        try {
            path = localPathfinder.generatePath(start, end, snapMode, additionalVerticies);
            System.out.println(path.size());
            hasRecievedPath = true;
            path.sendToSmartDashboard();
        } catch (Exception e) {
            e.printStackTrace();
            error = true;
        }
    }
}