package frc.robot.constants;

import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;

public class FieldConstants {
    public static final Translation3d ALGEA_OUTPUT_LOCATION = new Translation3d(6, 0, 0.41);
    public static final Translation3d ALGEA_OUTPUT_TOLERANCE = new Translation3d(6, 0, 0.41);

    public static final Translation2d BLUE_START_LOCATION = new Translation2d(Meter.of(2.5), Meter.of(4));
    public static final Translation2d RED_START_LOCATION = new Translation2d(Meter.of(14.5), Meter.of(4));

    // Dosya yolu
    private static final String layoutPath = Filesystem.getDeployDirectory() + "/apriltags/2026-rebuilt-welded.json";
    
    // Yardımcı metot ile güvenli yükleme
    public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = loadAprilTagLayout();

    private static AprilTagFieldLayout loadAprilTagLayout() {
        try {
            // HATA BURADAYDI: loadField yerine doğrudan constructor kullanıyoruz
            return new AprilTagFieldLayout(layoutPath);
        } catch (IOException e) {
            // Eğer dosya bulunamazsa terminale hata basar
            System.err.println("2026 AprilTag dosyası yüklenemedi! Yol: " + layoutPath);
            e.printStackTrace();
            return null; 
        }
    }
}