import javax.swing.*;
import java.awt.*;
import java.net.DatagramPacket;
import java.net.DatagramSocket;

public class TagCoordinatesViewer extends JFrame {
    private CoordinatePanel coordinatePanel;

    public TagCoordinatesViewer() {
        super("Tag Coordinates Viewer");
        coordinatePanel = new CoordinatePanel();
        add(coordinatePanel);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setSize(1000, 1000);
        setLocationRelativeTo(null);
        setVisible(true);
    }

    public void startServer() {
        try {
            DatagramSocket serverSocket = new DatagramSocket(3000);
            byte[] receiveData = new byte[1024];

            while (true) {
                DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
                serverSocket.receive(receivePacket);

                String receivedMessage = new String(receivePacket.getData());

                //UDP 소켓 통신으로 수신한 x,y 좌표를 정규식 사용하여 분리 -> 배열에 담음
                double[] coord = new double[3];
                double[] finalXY = new double[2];
                coord[0] = Double.parseDouble(receivedMessage.substring(3, 7)) * 100;
                coord[1] = Double.parseDouble(receivedMessage.substring(12, 16)) * 100;
                coord[2] = Double.parseDouble(receivedMessage.substring(21, 25)) * 100;

                //System.out.println(coord[0]+ " "+coord[1]+ " "+coord[2]);

                Trilateration tr = new Trilateration();
                finalXY = tr.trilaterate(coord[0], coord[1], coord[2]);
                //System.out.println(finalXY[0]+" "+finalXY[1]);

                //배열의 길이가 2 -> 즉 x,y 좌표 정상적으로 수신했으면, 파싱 후 double형으로 타입캐스팅
                if (finalXY.length == 2) {
//                    double x = Double.parseDouble(coordinates[0].substring(3));
//                    double y = Double.parseDouble(coordinates[1].substring(3));

                    // UI 출력 클래스의 setter에 x, y 좌표를 할당
                    coordinatePanel.setCoordinates((int) finalXY[0], (int) finalXY[1]);
                    System.out.println("x : "+ coordinatePanel.getX() + " / y :" + coordinatePanel.getY());

                    //coordinatePanel.paintComponent(getGraphics());

                    //출력
                    coordinatePanel.paint(getGraphics());
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {
//        SwingUtilities.invokeLater(() -> {
//            TagCoordinatesViewer viewer = new TagCoordinatesViewer();
//            viewer.startServer();
//        });
        TagCoordinatesViewer tc = new TagCoordinatesViewer();
        tc.startServer();
    }

    private class CoordinatePanel extends JPanel {
        private final int offset = 200;
        double anchor1X = 250.0 + offset;
        double anchor1Y = 0.0 + offset;
        double anchor2X = 0.0 + offset;
        double anchor2Y = 200.0 + offset;
        double anchor3X = 0.0 + offset;
        double anchor3Y = 0.0 + offset;
        private int x;
        private int y;

        public void setCoordinates(int x, int y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public int getX() {
            return x;
        }

        @Override
        public int getY() {
            return y;
        }

        public CoordinatePanel() {
            setBackground(Color.WHITE);
        }

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            int radius = 10;
            g.setColor(Color.BLACK);
            g.fillOval((int) anchor1X - radius, (int) anchor1Y - radius, 2 * radius, 2 * radius);
            g.setColor(Color.BLACK);
            g.fillOval((int) anchor2X - radius, (int) anchor2Y - radius, 2 * radius, 2 * radius);
            g.setColor(Color.BLACK);
            g.fillOval((int) anchor3X - radius, (int) anchor3Y - radius, 2 * radius, 2 * radius);
            g.setColor(Color.RED);
//            int xPos = getWidth() / 2 - radius + x;
//            int yPos = getHeight() / 2 - radius - y;
            g.fillOval(getX() + offset, getY() + offset, 2 * radius, 2 * radius);
        }
    }

    class Trilateration {

        private double x1 =250.0;
        private double y1 = 0.0;
        private double x2 = 0.0;
        private double y2 = 250.0;
        private double x3 = 0.0;
        private double y3 = 0.0;


        public double[] trilaterate(double d1, double d2,double d3) {

            double[] finalxy = new double[2];
            // Calculate squared distances
            double d1Squared = d1 * d1;
            double d2Squared = d2 * d2;
            double d3Squared = d3 * d3;

            // Calculate differences in anchor coordinates
            double x1_x2 = x1 - x2;
            double x1_x3 = x1 - x3;
            double y1_y2 = y1 - y2;
            double y1_y3 = y1 - y3;

            // Calculate constants
            double a = -2.0 * x1_x2;
            double b = -2.0 * y1_y2;
            double c = d2Squared - d1Squared - x2 * x2 + x1 * x1 - y2 * y2 + y1 * y1;
            double d = -2.0 * x1_x3;
            double e = -2.0 * y1_y3;
            double f = d3Squared - d1Squared - x3 * x3 + x1 * x1 - y3 * y3 + y1 * y1;

            // Calculate coordinates using Cramer's rule
            double denominator = a * e - b * d;
            if (denominator != 0.0) {
                double x = (c * e - b * f) / denominator;
                double y = (a * f - c * d) / denominator;
                //소수점 둘째자리까지 자르기 위해서 100을 곱하고 100.0을 나눠준다. double로 리턴하기 위해 100.0으로 나눔
                //부호가 바뀌어서 나오기 때문에 -1을 곱해준다.
                finalxy[0] = Math.round((x * -1)* 100) / 100.0;
                finalxy[1] = Math.round((y * -1)* 100) / 100.0;


                return finalxy;

            } else {
                // If denominator is zero, the anchors are in a straight line
                return null;
            }
        }


    }
}
