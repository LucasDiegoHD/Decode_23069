package org.firstinspires.ftc.teamcode.utils;

import android.os.Environment;
import com.pedropathing.geometry.Pose;
import java.io.*;

public class PoseStorage {
    private static final String FILE_PATH = Environment.getExternalStorageDirectory().getPath() + "/stored_robot_pose.txt";

    public static void savePose(Pose pose) {
        if (pose == null) return;
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(FILE_PATH))) {
            // Salva no formato: X,Y,Heading
            writer.write(pose.getX() + "," + pose.getY() + "," + pose.getHeading());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static Pose loadPose() {
        File file = new File(FILE_PATH);
        if (!file.exists()) return null;

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line = reader.readLine();
            if (line != null) {
                String[] tokens = line.split(",");
                if (tokens.length == 3) {
                    return new Pose(
                            Double.parseDouble(tokens[0]),
                            Double.parseDouble(tokens[1]),
                            Double.parseDouble(tokens[2])
                    );
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return null;
    }
}