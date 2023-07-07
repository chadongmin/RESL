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
                String[] coordinates = receivedMessage.trim().split(",");

                //배열의 길이가 2 -> 즉 x,y 좌표 정상적으로 수신했으면, 파싱 후 double형으로 타입캐스팅
                if (coordinates.length == 2) {
                    double x = Double.parseDouble(coordinates[0].substring(3));
                    double y = Double.parseDouble(coordinates[1].substring(3));

                    // UI 출력 클래스의 setter에 x, y 좌표를 할당
                    coordinatePanel.setCoordinates((int)x, (int)y);
                    System.out.println(coordinatePanel.getX()+" / "+coordinatePanel.getY());

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
        SwingUtilities.invokeLater(() -> {
            TagCoordinatesViewer viewer = new TagCoordinatesViewer();
            viewer.startServer();
        });
    }

    private class CoordinatePanel extends JPanel {
        private final int Edit = 100;
        double anchor1X = 0.0 + Edit;
        double anchor1Y = 0.0+ Edit;
        double anchor2X = 0.0+ Edit;
        double anchor2Y = 260.0+ Edit;
        double anchor3X = 400.0+ Edit;
        double anchor3Y = 260+ Edit;
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
            g.fillOval(getX()+Edit, getY()+Edit, 2 * radius, 2 * radius);
        }



    }
}
