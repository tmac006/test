// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/DrivebaseSubsystem.h"

#include <choreo/lib/ChoreoSwerveCommand.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>
#include <frc2/command/ProfiledPIDCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include <filesystem>

#include "choreo/lib/Choreo.h"
#include "frc/Filesystem.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/CommandPtr.h"
#include "str/Units.h"

DrivebaseSubsystem::DrivebaseSubsystem()
    : choreoController(choreolib::Choreo::ChoreoSwerveController(
          xTranslationController, yTranslationController, rotationController)) {
  SetupAutoBuilder();
  LoadChoreoTrajectories();
  thetaController.EnableContinuousInput(-180_deg, 180_deg);
}

void DrivebaseSubsystem::SetupAutoBuilder() {
  pathplanner::AutoBuilder::configureHolonomic(
      [this] { return swerveDrive.GetPose(); },
      [this](frc::Pose2d resetPose) {
        swerveDrive.SetGyroYaw(resetPose.Rotation().Radians());
        swerveDrive.SeedFieldRelative(resetPose);
      },
      [this] { return swerveDrive.GetRobotRelativeSpeeds(); },
      [this](frc::ChassisSpeeds speeds) {
        swerveDrive.SetChassisSpeeds(speeds, false);
      },
      pathplanner::HolonomicPathFollowerConfig(
          pathplanner::PIDConstants{
              constants::swerve::pathplanning::TRANSLATION_P,
              constants::swerve::pathplanning::TRANSLATION_I,
              constants::swerve::pathplanning::TRANSLATION_D},
          pathplanner::PIDConstants{
              constants::swerve::pathplanning::ROTATION_P,
              constants::swerve::pathplanning::ROTATION_I,
              constants::swerve::pathplanning::ROTATION_D},
          constants::swerve::physical::MAX_LINEAR_SPEED_FOC,
          swerveDrive.driveRadius,
          pathplanner::ReplanningConfig{true, true, .25_m, .25_m}),
      [this] { return ShouldMirrorPath(); }, this);
}

bool DrivebaseSubsystem::ShouldMirrorPath() {
  auto allyValue = frc::DriverStation::GetAlliance();
  if (allyValue) {
    if (allyValue.value() == frc::DriverStation::Alliance::kRed) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool DrivebaseSubsystem::HavePIDsChanged(
    units::scalar_t transP, units::scalar_t transI, units::scalar_t transD,
    units::scalar_t rotP, units::scalar_t rotI, units::scalar_t rotD) {
  return !units::essentiallyEqual(
             units::scalar_t{xTranslationController.GetP()}, transP, 1e-6) ||
         !units::essentiallyEqual(
             units::scalar_t{xTranslationController.GetI()}, transI, 1e-6) ||
         !units::essentiallyEqual(
             units::scalar_t{xTranslationController.GetD()}, transD, 1e-6) ||
         !units::essentiallyEqual(units::scalar_t{rotationController.GetP()},
                                  rotP, 1e-6) ||
         !units::essentiallyEqual(units::scalar_t{rotationController.GetI()},
                                  rotI, 1e-6) ||
         !units::essentiallyEqual(units::scalar_t{rotationController.GetD()},
                                  rotD, 1e-6);
}

// This method will be called once per scheduler run
void DrivebaseSubsystem::Periodic() {
  if (pathTuning) {
    double newTransP = frc::SmartDashboard::GetNumber("Drivebase/TRANS_P", 0);
    double newTransI = frc::SmartDashboard::GetNumber("Drivebase/TRANS_I", 0);
    double newTransD = frc::SmartDashboard::GetNumber("Drivebase/TRANS_D", 0);
    double newRotP = frc::SmartDashboard::GetNumber("Drivebase/ROT_P", 0);
    double newRotI = frc::SmartDashboard::GetNumber("Drivebase/ROT_I", 0);
    double newRotD = frc::SmartDashboard::GetNumber("Drivebase/ROT_D", 0);
    if (HavePIDsChanged(newTransP, newTransI, newTransD, newRotP, newRotI,
                        newRotD)) {
      SetTranslationPIDs(newTransP, newTransI, newTransD);
      SetRotationPIDs(newRotP, newRotI, newRotD);
    }
  }
  units::meter_t distance = CalcDistanceFromSpeaker();
  swerveDrive.Log();
}

void DrivebaseSubsystem::SimulationPeriodic() {
  swerveDrive.SimulationUpdate();
}

void DrivebaseSubsystem::UpdateOdometry() {
  swerveDrive.UpdateOdometry();
}

void DrivebaseSubsystem::LoadChoreoTrajectories() {
  for (const auto& entry : std::filesystem::directory_iterator(
           frc::filesystem::GetDeployDirectory() + "/choreo/")) {
    std::string fileName = entry.path().stem().string();
    fmt::print("Loaded choreo trajectory: {}\n", fileName);
    pathMap[fileName] = choreolib::Choreo::GetTrajectory(fileName);
  }
}

void DrivebaseSubsystem::SetTranslationPIDs(double p, double i, double d) {
  xTranslationController.SetPID(p, i, d);
  yTranslationController.SetPID(p, i, d);
}

void DrivebaseSubsystem::SetRotationPIDs(double p, double i, double d) {
  rotationController.SetPID(p, i, d);
}

void DrivebaseSubsystem::SetPathTuning(bool onOff) {
  pathTuning = onOff;
}

frc2::CommandPtr DrivebaseSubsystem::ResetPosition(
    std::function<frc::Pose2d()> newPosition) {
  return frc2::cmd::RunOnce([this, newPosition] {
    swerveDrive.TareEverything();
    swerveDrive.SeedFieldRelative(newPosition());
  });
}

frc2::CommandPtr DrivebaseSubsystem::DriveFactory(
    std::function<double()> fow, std::function<double()> side,
    std::function<double()> rot, std::function<bool()> fieldOriented) {
  return frc2::RunCommand(
             [this, fow, side, rot, fieldOriented]() {
               swerveDrive.Drive(
                   fow() * constants::swerve::physical::MAX_LINEAR_SPEED,
                   side() * constants::swerve::physical::MAX_LINEAR_SPEED,
                   rot() * constants::swerve::physical::MAX_ROTATION_SPEED,
                   true, fieldOriented());
             },
             {this})
      .ToPtr()
      .WithName("Drive Factory");
}

frc2::CommandPtr DrivebaseSubsystem::TurnToAngleFactory(
    std::function<double()> fow, std::function<double()> side,
    std::function<frc::TrapezoidProfile<units::radians>::State()> angle,
    std::function<bool()> wantsToOverride, bool fieldRelative) {
  return frc2::ProfiledPIDCommand<units::radians>(
             thetaController,
             [this] { return swerveDrive.GetHeading().Radians(); }, angle,
             [this, fow, side, wantsToOverride, fieldRelative](
                 double output,
                 frc::TrapezoidProfile<units::radians>::State state) {
               swerveDrive.Drive(
                   fow() * constants::swerve::physical::MAX_LINEAR_SPEED,
                   side() * constants::swerve::physical::MAX_LINEAR_SPEED,
                   output * 1_rad_per_s, fieldRelative);
             },
             {this})
      .Until(wantsToOverride)
      .WithName("Turn To Angle Factory");
}

frc2::CommandPtr DrivebaseSubsystem::XCommand() {
  return frc2::cmd::Run(
             [this] {
               std::array<frc::SwerveModuleState, 4> states;

               states[0].angle = 45_deg;
               states[1].angle = -45_deg;
               states[2].angle = -45_deg;
               states[3].angle = 45_deg;

               swerveDrive.SetModuleStates(states, true);
             },
             {this})
      .WithName("X Pattern");
}

frc2::CommandPtr DrivebaseSubsystem::SelfTest() {
  return swerveDrive.SelfTest({this});
}

frc2::CommandPtr DrivebaseSubsystem::MeasureWheelDiam(
    std::function<bool()> done) {
  return swerveDrive.MeasureWheelDiam(done, {this});
}

frc2::CommandPtr DrivebaseSubsystem::TuneSteerPID(std::function<bool()> done) {
  return swerveDrive.TuneSteerPID(done, {this});
}

frc2::CommandPtr DrivebaseSubsystem::TuneDrivePID(std::function<bool()> done) {
  return swerveDrive.TuneDrivePID(done, {this});
}

frc2::CommandPtr DrivebaseSubsystem::ZeroYawCMD() {
  return frc2::cmd::RunOnce([this] { swerveDrive.ZeroYaw(); });
}

frc2::CommandPtr DrivebaseSubsystem::FollowChoreoTrajectory(
    std::function<std::string()> pathName) {
  return frc2::cmd::Either(
      frc2::cmd::Sequence(
          frc2::cmd::RunOnce([this, pathName] {
            if (ShouldMirrorPath()) {
              choreolib::ChoreoTrajectory flippedTraj =
                  pathMap[pathName()].Flipped();
              swerveDrive.SeedFieldRelative(flippedTraj.GetInitialPose());
              swerveDrive.GetField()
                  .GetObject("CurrentChoreoTrajectory")
                  ->SetPoses(flippedTraj.GetPoses());
            } else {
              swerveDrive.SeedFieldRelative(
                  pathMap[pathName()].GetInitialPose());
              swerveDrive.GetField()
                  .GetObject("CurrentChoreoTrajectory")
                  ->SetPoses(pathMap[pathName()].GetPoses());
            }
          }),
          choreolib::Choreo::ChoreoSwerveCommandFactory(
              pathMap[pathName()], [this] { return swerveDrive.GetPose(); },
              choreoController,
              [this](frc::ChassisSpeeds speeds) {
                swerveDrive.SetChassisSpeeds(speeds, false);
              },
              [this] { return ShouldMirrorPath(); }, {this}),
          frc2::cmd::RunOnce(
              [this] { swerveDrive.Drive(0_mps, 0_mps, 0_rad_per_s, false); })),
      frc2::cmd::Print(
          "ERROR: Choreo path wasn't found in pathMap!!!!\n\n\n\n"),
      [this, pathName] { return pathMap.contains(pathName()); });
}

frc2::CommandPtr DrivebaseSubsystem::TunePathPid() {
  return frc2::cmd::Sequence(frc2::cmd::RunOnce([this] {
    SetPathTuning(true);
    frc::SmartDashboard::PutNumber("Drivebase/TRANS_P",
                                   xTranslationController.GetP());
    frc::SmartDashboard::PutNumber("Drivebase/TRANS_I",
                                   xTranslationController.GetI());
    frc::SmartDashboard::PutNumber("Drivebase/TRANS_D",
                                   xTranslationController.GetD());
    frc::SmartDashboard::PutNumber("Drivebase/ROT_P",
                                   rotationController.GetP());
    frc::SmartDashboard::PutNumber("Drivebase/ROT_I",
                                   rotationController.GetI());
    frc::SmartDashboard::PutNumber("Drivebase/ROT_D",
                                   rotationController.GetD());
  }));
}

frc2::CommandPtr DrivebaseSubsystem::DoneTuningPathPids() {
  return frc2::cmd::RunOnce([this] { SetPathTuning(false); });
}

void DrivebaseSubsystem::AddVisionMeasurement(
    const frc::Pose2d& visionMeasurement, units::second_t timestamp) {
  swerveDrive.AddVisionMeasurement(visionMeasurement, timestamp);
}

void DrivebaseSubsystem::AddVisionMeasurement(
    const frc::Pose2d& visionMeasurement, units::second_t timestamp,
    const Eigen::Vector3d& stdDevs) {
  swerveDrive.GetField()
      .GetObject("Vision Pose To Add")
      ->SetPose(visionMeasurement);
  swerveDrive.AddVisionMeasurement(visionMeasurement, timestamp, stdDevs);
}

frc::Pose2d DrivebaseSubsystem::GetRobotPose() {
  return swerveDrive.GetPose();
}

frc::Pose2d DrivebaseSubsystem::GetOdomPose() {
  return swerveDrive.GetOdomPose();
}

frc2::CommandPtr DrivebaseSubsystem::GoToPose(
    std::function<frc::Pose2d()> poseToGoTo) {
  return frc2::cmd::RunOnce(
             [this, poseToGoTo] {
               xTranslationController.Reset();
               yTranslationController.Reset();
               rotationController.Reset();
               rotationController.EnableContinuousInput(-std::numbers::pi,
                                                        std::numbers::pi);
               xTranslationController.SetSetpoint(poseToGoTo().X().value());
               yTranslationController.SetSetpoint(poseToGoTo().Y().value());
               rotationController.SetSetpoint(
                   poseToGoTo().Rotation().Radians().value());
               lastPoseInMoveToArc = poseToGoTo();
               xTranslationController.SetTolerance(0.0762);
               yTranslationController.SetTolerance(0.0762);
               rotationController.SetTolerance(0.0349066);
               swerveDrive.GetField()
                   .GetObject("Pose to go to")
                   ->SetPose(poseToGoTo());
             },
             {this})
      .AndThen(frc2::cmd::Run(
          [this] {
            frc::Pose2d currentPose = GetRobotPose();
            units::meters_per_second_t xOutput{
                xTranslationController.Calculate(currentPose.X().value())};
            units::meters_per_second_t yOutput{
                yTranslationController.Calculate(currentPose.Y().value())};
            units::radians_per_second_t thetaOutput{
                rotationController.Calculate(
                    currentPose.Rotation().Radians().value())};
            auto allyValue = frc::DriverStation::GetAlliance();
            bool fieldOriented = true;
            if (allyValue) {
              if (allyValue.value() == frc::DriverStation::Alliance::kRed) {
                xOutput = -xOutput;
                yOutput = -yOutput;
              }
            }
            swerveDrive.Drive(xOutput, yOutput, thetaOutput, false,
                              fieldOriented);
          },
          {this}))
      .Until([this] {
        return xTranslationController.AtSetpoint() &&
               yTranslationController.AtSetpoint() &&
               rotationController.AtSetpoint();
      });
}

frc2::CommandPtr DrivebaseSubsystem::SysIdQuasistaticSteer(
    frc2::sysid::Direction direction) {
  return sysIdRoutineSteer.Quasistatic(direction).BeforeStarting(
      [this] { ctre::phoenix6::SignalLogger::Start(); });
}

frc2::CommandPtr DrivebaseSubsystem::SysIdDynamicSteer(
    frc2::sysid::Direction direction) {
  return sysIdRoutineSteer.Dynamic(direction).BeforeStarting(
      [this] { ctre::phoenix6::SignalLogger::Start(); });
}

frc2::CommandPtr DrivebaseSubsystem::SysIdQuasistaticDrive(
    frc2::sysid::Direction direction) {
  return sysIdRoutineDrive.Quasistatic(direction).BeforeStarting(
      [this] { ctre::phoenix6::SignalLogger::Start(); });
}

frc2::CommandPtr DrivebaseSubsystem::SysIdDynamicDrive(
    frc2::sysid::Direction direction) {
  return sysIdRoutineDrive.Dynamic(direction).BeforeStarting(
      [this] { ctre::phoenix6::SignalLogger::Start(); });
}

frc2::CommandPtr DrivebaseSubsystem::WheelRadFwd() {
  return swerveDrive.WheelRadiusCmd({this}, 1.0);
}

frc2::CommandPtr DrivebaseSubsystem::WheelRadRev() {
  return swerveDrive.WheelRadiusCmd({this}, -1.0);
}