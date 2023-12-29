package org.firstinspires.ftc.teamcode.common.drive.localizer;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.swerve.SwerveModule;

import java.util.Arrays;
import java.util.function.DoubleSupplier;



///Modificat fata de original, fara localizer interface bcuz redundant

public class BetterSwerveLocalizer implements com.acmerobotics.roadrunner.localization.Localizer {
    public SwerveModule.SwerveModuleState[] modules;
    public DoubleSupplier imu;
    public Pose2d poseEstimate;
    public Pose2d pastPoseEstimate;

    public BetterSwerveLocalizer(DoubleSupplier i, SwerveModule... mods){
        modules = Arrays.stream(mods).map(SwerveModule::asState).toArray(SwerveModule.SwerveModuleState[]::new);
        imu = i;
        poseEstimate = new Pose2d();
        pastPoseEstimate = new Pose2d();
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        poseEstimate = pose2d;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return poseEstimate.minus(pastPoseEstimate);
    }

    public void periodic() {
        update();
    }

    public Pose getPos() {
        Pose2d pose = getPoseEstimate();
        return new Pose(pose.getY(), pose.getX(), -pose.getHeading());
    }

    public void setPos(Pose pose) {
        setPoseEstimate(new Pose2d(pose.y, pose.x, pose.heading));
    }

    @Override
    public void update() {
        pastPoseEstimate = poseEstimate;
        Vector2d accumulator = new Vector2d();
        double head = imu.getAsDouble();
        for(SwerveModule.SwerveModuleState s : modules){
            accumulator = accumulator.plus(s.calculateDelta());
        }
        accumulator = accumulator.div(modules.length).rotated(head);
        poseEstimate = new Pose2d(poseEstimate.vec().plus(accumulator), head);
    }

}