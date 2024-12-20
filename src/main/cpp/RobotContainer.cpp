// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
  (void)VisionConstants::kSyncBytes[0];
  RegisterAutoCommands();
  
  // Initialize all of your commands and subsystems here 

  // Grant default control of swerve drive to either bill (controller 0),
  // or test (controller 2), based on a constant
  switch (SwerveDriveConstants::kActiveController) {
    case(ActiveDriveController::OfficialDriver) :
      m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
      break;
    case(ActiveDriveController::TestControls) :
      m_swerveDrive->SetDefaultCommand(Drive(&m_test, m_swerveDrive, m_driveState));
      break;
    default :
      break;
  }

  frc::SmartDashboard::PutData("Auto Routines", &m_chooser);
  
  // m_intake->SetDefaultCommand(frc2::RunCommand([this] {
  //   m_intake->SetRotationPower(m_ted.GetRawAxis(OperatorConstants::kAxisLeftStickY));
  // },{m_intake}).ToPtr());

  // m_shooter->SetDefaultCommand(frc2::RunCommand([this]{
  //   auto input = m_ted.GetRawAxis(OperatorConstants::kAxisRightStickY);
  //   if (fabs(input) > 0.05)
  //     m_shooter->SetRotationPower(-m_ted.GetRawAxis(OperatorConstants::kAxisRightStickY));
  //   else
  //     m_shooter->SetRotationPower(0.02);
  //   // May need to change joystick axis
  // },{m_shooter}));
  
  // Configure the button bindings
  ConfigureDriverBindings();
  ConfigureCoDriverBindings();
  ConfigureTestBindings();
  // CreateAutoPaths();
}

void RobotContainer::ConfigureDriverBindings() {
  // Bill controls
  // Setting the Drive State (Normal Field relative and Rotational Velocity Control) to the desired one based on controller right stick
  // frc2::Trigger toggleATan2RotButton{m_bill.Button(OperatorConstants::kButtonIDRightStick)};
  // toggleATan2RotButton.OnTrue(
  //   frc2::InstantCommand([this]{
  //     if (m_driveState == DriveState::HeadingControl) {
  //       m_driveState = DriveState::RotationVelocityControl;
  //     } else if (m_driveState == DriveState::RotationVelocityControl) {
  //       m_driveState = DriveState::HeadingControl;
  //     }
  //     m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
  //   },{m_swerveDrive}).ToPtr()
  // );

  // creating swerve drive instance using the Drive command on the bill controller (main driving)

  //the left bumper will set the intake to be active by first extending it and then running it with a parrallel command group. Shooter goes to load position while this is happening

  // frc2::Trigger fullIntakeUnloadButton{m_bill.GetL1Button()};
  frc2::Trigger fullIntakeUnloadButton([this](){ return m_bill.GetL1Button(); });
  fullIntakeUnloadButton.OnTrue(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{ m_shooter->SetBrakeMode(BrakeMode::Brake); m_shooter->SetRollerPower(0.0); },{m_shooter}),
    frc2::ParallelCommandGroup(
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetIntakeRotation(m_intake, IntakeState::Extended)
      ),
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetShooterRotation(m_shooter, ShooterState::Load)
      )
    ),
    frc2::ParallelRaceGroup(
      RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0, ShooterEndCondition::EndOnFirstDetection),
      RunIntake(m_intake, IntakeConstants::kIntakePower)
    )
  ).ToPtr());
  fullIntakeUnloadButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kUnloadTime),
      RunLoader(m_shooter, ShooterConstants::kUnloadPower, ShooterConstants::kUnloadPower)
    ),
    frc2::InstantCommand([this]{
      m_shooter->SetBrakeMode(BrakeMode::Coast);
      m_intake->SetRollerPower(0.0);
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_intake, m_shooter}),
    frc2::ParallelCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::Zero),
      SetIntakeRotation(m_intake, IntakeState::Retracted)
    )
  ).ToPtr());

  //set the intake to get the piece using logic from the AmpIntake command to score in the amp
  // frc2::Trigger ampIntakeButton{m_bill.GetR1Button()};
  frc2::Trigger ampIntakeButton([this](){ return m_bill.GetR1Button(); });
  ampIntakeButton.OnTrue(frc2::SequentialCommandGroup(
    SetIntakeRotation(m_intake, IntakeState::Extended),
    AmpIntake(m_intake, IntakeConstants::kIntakePower)
  ).ToPtr());
  ampIntakeButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
    },{m_intake}),
    SetIntakeRotation(m_intake, IntakeState::Retracted)
  ).ToPtr());

  //align to the speaker when the left trigger is pressed
  // frc2::Trigger alignSpeakerButton{m_bill.Button(OperatorConstants::kButtonIDLeftTrigger)};
  // alignSpeakerButton.OnTrue(frc2::InstantCommand([this]{
  //   // if (m_driveState == DriveState::HeadingControl) {
  //     m_driveState = DriveState::SpeakerAlignTranslationAlgorithm;
  //   // } else {
  //   //   m_driveState = DriveState::HeadingControl;
  //   // }
  //     m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
  //   },{m_swerveDrive}).ToPtr()
  // );
  // alignSpeakerButton.OnFalse(frc2::InstantCommand([this]{
  //     m_driveState = DriveState::HeadingControl;
  //     m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
  //   },{m_swerveDrive}).ToPtr()
  // );

  // frc2::Trigger driveUnderStageButton{m_bill.Button(OperatorConstants::kButtonIDLeftTrigger)};
  // driveUnderStageButton.OnTrue(SetShooterRotation(m_shooter, ShooterState::Far).ToPtr());
  // driveUnderStageButton.OnFalse(SetShooterRotation(m_shooter, ShooterState::Zero).ToPtr());

  // score in the amp 
  frc2::Trigger ampScoreIntakeButton([this](){ return m_bill.GetR2Button(); });
  ampScoreIntakeButton.OnTrue(frc2::ParallelCommandGroup(
    // frc2::InstantCommand([this]{
    //   m_driveState = DriveState::ArbitraryAngleAlign;
    // //   //setting drive state to align to a an arbitrary angle
    // //   m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState, SwerveDriveConstants::kAmpAlignTarget));
    // },{}),
    // Drive(&m_bill, m_swerveDrive, DriveState::ArbitraryAngleAlign, SwerveDriveConstants::kAmpAlignTarget),
    SetIntakeRotation(m_intake, IntakeState::Amp)
  ).ToPtr());
  ampScoreIntakeButton.OnFalse(frc2::SequentialCommandGroup(
    // frc2::InstantCommand([this]{
    //   m_driveState = DriveState::HeadingControl;
    //   m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
    // },{m_swerveDrive}),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(IntakeConstants::kAmpShootTime), 
      RunIntake(m_intake, IntakeConstants::kAmpScorePower)
    ),
    SetIntakeRotation(m_intake, IntakeState::Retracted)
  ).ToPtr());

  // reset the pose of the robot in the case of noise or natural dampening
  frc2::Trigger resetPoseButton([this](){ return m_bill.GetTouchpadButton(); });
  resetPoseButton.OnTrue(frc2::InstantCommand([this]{
    if (frc::DriverStation::GetAlliance()) {
      // if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
      //   m_swerveDrive->SetPose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}}, true);
      // else
      //   m_swerveDrive->SetPose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{180.0_deg}}, true);
      if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
        m_swerveDrive->SetPose(frc::Pose2d{0.92_m, 5.50_m, frc::Rotation2d{0.0_deg}}, false);
      else
        m_swerveDrive->SetPose(frc::Pose2d{15.579_m, 5.50_m, frc::Rotation2d{180.0_deg}}, false);
    }
  },{m_swerveDrive}).ToPtr());

  // based on alliance, it will reset robot pose to the speaker position
  frc2::Trigger resetSpeakerPoseButton([this](){ return m_bill.GetCreateButton(); });
  resetSpeakerPoseButton.OnTrue(frc2::InstantCommand([this]{
    if (frc::DriverStation::GetAlliance()) {
      if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
        m_swerveDrive->SetPose(frc::Pose2d{0.92_m, 5.50_m, frc::Rotation2d{0.0_deg}}, false);
      else
        m_swerveDrive->SetPose(frc::Pose2d{15.579_m, 5.50_m, frc::Rotation2d{180.0_deg}}, false);
    }
  },{m_swerveDrive}).ToPtr());

  // resetSpeakerPoseButton.OnTrue(RunIntake(m_intake, 0.5).ToPtr());
  // resetSpeakerPoseButton.OnFalse(RunIntake(m_intake, 0.0).ToPtr());

  // enables climb mode
  frc2::Trigger toggleClimbModeButton([this](){ return m_bill.GetOptionsButton(); });
  toggleClimbModeButton.OnTrue(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      if (GetSuperstructureState() == SuperstructureState::Default) {
        m_superstructureState = SuperstructureState::Climb;
      } else {
        m_superstructureState = SuperstructureState::Default;
        // this is the overall state if the robot is driving or climbing
      }
    }),
    frc2::ParallelCommandGroup(
      SetIntakeRotation(m_intake, IntakeState::Extended),
      SetShooterRotation(m_shooter, ShooterState::Zero)
      // shooter and intake need to be in these positions while climb
    )
  ).ToPtr());

  // frc2::Trigger climbConfigButton{m_bill.Get)};
  // climbConfigButton.OnTrue(frc2::ParallelCommandGroup(
  //   frc2::InstantCommand([this]{ m_swerveDrive->ToggleSlowMode(); },{m_swerveDrive}),
  //   SetIntakeRotation(m_intake, IntakeState::Extended),
  //   SetShooterRotation(m_shooter, ShooterState::Zero)
  // ).ToPtr());
  
  // frc2::Trigger extendIntakeButton{m_bill.Button(OperatorConstants::kButtonIDTriangle)};
  // extendIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeState::Extended).ToPtr());

  // frc2::Trigger retractIntakeButton{m_bill.Button(OperatorConstants::kButtonIDX)};
  // retractIntakeButton.OnTrue(SetIntakeRotation(m_intake, IntakeState::Retracted).ToPtr());

  // frc2::Trigger runIntake{m_bill.Button(OperatorConstants::kButtonIDCircle)};
  // runIntake.OnTrue(frc2::InstantCommand([this]{
  //     m_intake->SetRollerPower(1.0);
  // },{m_intake}).ToPtr());
  // runIntake.OnFalse(frc2::InstantCommand([this]{
  //     m_intake->SetRollerPower(0.0);
  // },{m_intake}).ToPtr());

  // frc2::Trigger toggleSlowModeButton{m_bill.GetR1Button()};
  // toggleSlowModeButton.OnTrue(frc2::InstantCommand([this]{ m_swerveDrive->ToggleSlowMode(); },{m_swerveDrive}).ToPtr());
}

void RobotContainer::ConfigureCoDriverBindings() {
  // Ted controls
  // human player load controls: sets target to the human player source based on alliance
  frc2::Trigger directShooterLoadButton([this](){ return m_ted.GetL1Button(); });
  directShooterLoadButton.OnTrue(frc2::ParallelCommandGroup(
    Drive(&m_bill, m_swerveDrive, DriveState::SourceAlign),
    // Drive(&m_bill, m_swerveDrive, DriveState::HeadingControl, m_driveAligntarget),
    frc2::SequentialCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::DirectLoad),
      RunLoader(m_shooter, ShooterConstants::kDirectLoadPower, ShooterConstants::kDirectLoadPower, ShooterEndCondition::None),
      frc2::ParallelCommandGroup(
        SetShooterRotation(m_shooter, ShooterState::Zero),
        SetIntakeRotation(m_intake, IntakeState::Retracted)
      )
    )
  ).ToPtr());
  directShooterLoadButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_driveState = DriveState::HeadingControl;
      m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
    },{m_swerveDrive}),
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter}),
    frc2::ParallelCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::Zero),
      SetIntakeRotation(m_intake, IntakeState::Retracted)
    )
    // regular drive state if we don't want to directly load
  ).ToPtr());

  frc2::Trigger unloadButton([this](){ return m_ted.GetR1Button(); });
  unloadButton.OnTrue(RunLoader(m_shooter, ShooterConstants::kUnloadPower, ShooterConstants::kUnloadPower).ToPtr());
  unloadButton.OnFalse(
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter}).ToPtr()
  );

  // frc2::Trigger autoShootButton{m_ted.Button(OperatorConstants::kButtonIDLeftTrigger)};
  // autoShootButton.OnTrue(ShooterAutoAlign(m_shooter, m_poseEstimator, m_vision, false).ToPtr());
  // autoShootButton.OnFalse(frc2::SequentialCommandGroup(
  //   frc2::ParallelDeadlineGroup(
  //     frc2::WaitCommand(ShooterConstants::kShootTime),
  //     RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
  //   ),
  //   SetShooterRotation(m_shooter, ShooterState::Zero),
  //   frc2::InstantCommand([this]{
  //     m_shooter->SetRollerPower(0.0);
  //     m_shooter->SetLoaderPower(0.0);
  //   },{m_shooter})
  // ).ToPtr());

  //all the below shoot buttons set the shooter rotation to the desired target based on if shooteer far, mid, close, manually, or auto and then runs it.
  //shooter will run at max power and angle changes trajectory

  // frc2::Trigger shootLowButton{m_ted.Button(OperatorConstants::kButtonIDLeftTrigger)};
  // shootLowButton.OnTrue(frc2::SequentialCommandGroup(
  //   SetShooterRotation(m_shooter, ShooterState::Far),
  //   RunShooter(m_shooter, ShooterConstants::kShootPower)
  // ).ToPtr());
  // shootLowButton.OnFalse(frc2::SequentialCommandGroup(
  //   frc2::ParallelDeadlineGroup(
  //     frc2::WaitCommand(ShooterConstants::kShootTime),
  //     RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
  //   ),
  //   frc2::InstantCommand([this]{

  //     m_shooter->SetRollerPower(0.0);
  //     m_shooter->SetLoaderPower(0.0);
  //   },{m_shooter})
  // ).ToPtr());
  
  frc2::Trigger closeShootButton([this](){ return m_ted.GetR2Button(); });
  closeShootButton.OnTrue(frc2::SequentialCommandGroup(
    SetShooterRotation(m_shooter, ShooterState::Close),
    RunShooter(m_shooter, ShooterConstants::kShootPower)
  ).ToPtr());
  closeShootButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kShootTime),
      RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
    ),
    SetShooterRotation(m_shooter, ShooterState::Zero),
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter})
  ).ToPtr());

  frc2::Trigger closeFastShootButton([this](){ return m_ted.GetCircleButton(); });
  closeFastShootButton.OnTrue(frc2::SequentialCommandGroup(
    SetShooterRotation(m_shooter, ShooterState::Close),
    RunShooter(m_shooter, 1.0)
  ).ToPtr());
  closeFastShootButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kShootTime),
      RunLoader(m_shooter, 1.0, 1.0)
    ),
    SetShooterRotation(m_shooter, ShooterState::Zero),
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter})
  ).ToPtr());

  frc2::Trigger articulateShooterButton([this](){ return m_ted.GetL2Button(); });
  articulateShooterButton.OnTrue(
    SetShooterRotation(m_shooter, ShooterState::StartingAuto)
  .ToPtr());
  articulateShooterButton.OnFalse(
    SetShooterRotation(m_shooter, ShooterState::Zero)
  .ToPtr());
  // frc2::Trigger midShootButton{m_ted.Button(OperatorConstants::kButtonIDTriangle)};
  // midShootButton.OnTrue(frc2::SequentialCommandGroup(
  //   SetShooterRotation(m_shooter, ShooterState::Mid),
  //   RunShooter(m_shooter, ShooterConstants::kShootPower)
  // ).ToPtr());
  // midShootButton.OnFalse(frc2::SequentialCommandGroup(
  //   frc2::ParallelDeadlineGroup(
  //     frc2::WaitCommand(ShooterConstants::kShootTime),
  //     RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
  //   ),
  //   SetShooterRotation(m_shooter, ShooterState::Zero),
  //   frc2::InstantCommand([this]{
  //     m_shooter->SetRollerPower(0.0);
  //     m_shooter->SetLoaderPower(0.0);
  //   },{m_shooter})
  // ).ToPtr());

  // frc2::Trigger manualShootButton{m_ted.Button(OperatorConstants::kButtonIDCircle)};
  // manualShootButton.OnTrue(RunShooter(m_shooter, ShooterConstants::kShootPower).ToPtr());
  // manualShootButton.OnFalse(frc2::SequentialCommandGroup(
  //   frc2::ParallelDeadlineGroup(
  //     frc2::WaitCommand(ShooterConstants::kShootTime),
  //     RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
  //   ),
  //   frc2::InstantCommand([this]{
  //     m_shooter->SetRollerPower(0.0);
  //     m_shooter->SetLoaderPower(0.0);
  //   },{m_shooter})
  // ).ToPtr());
  
  frc2::Trigger extendBothClimbersButton([this](){ return m_ted.GetTriangleButton(); });
  extendBothClimbersButton.OnTrue(frc2::SequentialCommandGroup(
    RunClimber(m_climber, ClimberConstants::kBothExtendPower, ClimberConstants::kBothExtendPower)
  )
    // .OnlyIf([this](){ return IsClimbState(); }));
    .ToPtr());
  extendBothClimbersButton.OnFalse(frc2::InstantCommand([this]{
      m_climber->SetPower(0.0, 0.0);
  },{m_climber})
    // .OnlyIf([this](){ return IsClimbState(); }));
    .ToPtr());

  frc2::Trigger retractBothClimbersButton([this](){ return m_ted.GetCrossButton(); });
  retractBothClimbersButton.OnTrue(RunClimber(m_climber, ClimberConstants::kBothRetractPower, ClimberConstants::kBothRetractPower)
    // .OnlyIf([this](){ return IsClimbState(); }));
    .ToPtr());
  retractBothClimbersButton.OnFalse(frc2::InstantCommand([this]{
      m_climber->SetPower(0.0, 0.0);
  },{m_climber})
    // .OnlyIf([this](){ return IsClimbState(); }));
    .ToPtr());
    
  // frc2::Trigger shooterZeroButton{m_ted.Button(OperatorConstants::kButtonIDTriangle)};
  // shooterZeroButton.OnTrue(SetShooterRotation(m_shooter, ShooterState::Zero).ToPtr());

  // frc2::Trigger shooterCloseRangeButton{m_ted.Button(OperatorConstants::kButtonIDX)};
  // shooterCloseRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterState::Close).ToPtr());

  // frc2::Trigger shooterMidRangeButton{m_ted.Button(OperatorConstants::kButtonIDCircle)};
  // shooterMidRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterState::Mid).ToPtr());

  // frc2::Trigger shooterFarRangeButton{m_ted.Button(OperatorConstants::kButtonIDSquare)};
  // shooterFarRangeButton.OnTrue(SetShooterRotation(m_shooter, ShooterState::Far).ToPtr());
}

void RobotContainer::RegisterAutoCommands() {
  // Start of Auto Events
  pathplanner::NamedCommands::registerCommand("AlignShooter", frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(AutoConstants::kAutoUnloadTime),
      RunLoader(m_shooter, AutoConstants::kAutoUnloadPower, 0.0, ShooterEndCondition::None)
    ),
    ShooterAutoAlign(m_shooter, m_poseEstimator, m_vision, true)
  ).ToPtr());

  pathplanner::NamedCommands::registerCommand("AlignSwerve", TurnInPlace(m_swerveDrive, DriveState::SpeakerAlignTranslationAlgorithm).ToPtr());

  pathplanner::NamedCommands::registerCommand("PrintAutoMessage", frc2::InstantCommand([this]{
    for (int i = 0; i < 10; i++) {
      std::cout << "Auto started/ended\n"; }},{}).ToPtr());

  pathplanner::NamedCommands::registerCommand("Intake", frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{ m_shooter->SetBrakeMode(BrakeMode::Brake); m_shooter->SetRollerPower(0.0); },{m_shooter}),
    frc2::InstantCommand([this]{ m_intake->SetRollerPower(IntakeConstants::kIntakePower); },{m_intake}),
    frc2::ParallelCommandGroup(
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetIntakeRotation(m_intake, IntakeState::Extended)
      ),
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetShooterRotation(m_shooter, ShooterState::Load)
      )
    ),
    frc2::ParallelRaceGroup(
      frc2::WaitCommand(AutoConstants::kIntakeDuration),
      RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0, ShooterEndCondition::EndOnFirstDetection),
      RunIntake(m_intake, IntakeConstants::kIntakePower)
    ),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kUnloadTime),
      RunLoader(m_shooter, ShooterConstants::kUnloadPower, ShooterConstants::kUnloadPower)
    ),
    frc2::InstantCommand([this]{
      m_intake->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_intake, m_shooter})
    // frc2::ParallelCommandGroup(
    //   SetShooterRotation(m_shooter, ShooterState::Zero),
    //   SetIntakeRotation(m_intake, IntakeState::Retracted)
    // )
  ).ToPtr());

  pathplanner::NamedCommands::registerCommand("ImmediateShoot", frc2::SequentialCommandGroup(
    frc2::ParallelCommandGroup(
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetIntakeRotation(m_intake, IntakeState::Extended)
      ),
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetShooterRotation(m_shooter, ShooterState::Far)
      )
    ),
    frc2::ParallelCommandGroup(
      RunIntake(m_intake, IntakeConstants::kIntakePower),
      RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
    )
  ).ToPtr());

  // First, unload the note briefly so that it isn't stuck in the shooter's wheels
  // Raises the shooter to the correct angle to fire from the front of the subwoofer
  // Also ramps up the shooter's velocity at the same time to prepare a shot
  pathplanner::NamedCommands::registerCommand("PrepareShooterSubwoofer", frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(AutoConstants::kAutoUnloadTime),
      RunLoader(m_shooter, AutoConstants::kAutoUnloadPower, 0.0, ShooterEndCondition::None)
    ),
    frc2::InstantCommand([this]{ m_shooter->SetRollerPower(ShooterConstants::kShootPower); },{m_shooter}),
    // Also ramps up the speed while lifting the shooter
    SetShooterRotation(m_shooter, ShooterState::AutoScore)
  ).ToPtr());

  pathplanner::NamedCommands::registerCommand("RotateTo90", TurnInPlace(m_swerveDrive, DriveState::ArbitraryAngleAlign, 90.0_deg).ToPtr());

  pathplanner::NamedCommands::registerCommand("RotateToNegative30", TurnInPlace(m_swerveDrive, DriveState::ArbitraryAngleAlign, -30.0_deg).ToPtr());

  pathplanner::NamedCommands::registerCommand("ScoreSpeaker", frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(AutoConstants::kAutoRevUpTime),
      RunShooter(m_shooter, ShooterConstants::kShootPower)
    ),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kShootTime),
      RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
    )
    // frc2::ParallelCommandGroup(
    //   SetShooterRotation(m_shooter, ShooterState::Zero),
    //   SetIntakeRotation(m_intake, IntakeState::Retracted)
    // )
  ).ToPtr());

  pathplanner::NamedCommands::registerCommand("ShooterRotateToAutoTarget", frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{ m_shooter->SetRollerPower(ShooterConstants::kShootPower); },{m_shooter}),
    SetShooterRotation(m_shooter, ShooterState::AutoScore)
  ).ToPtr());
} 

void RobotContainer::CreateAutoPaths() {
  m_chooser.SetDefaultOption("N/A", nullptr);\
  for (auto autoPath : AutoConstants::kAutonomousPaths) {
    m_chooser.AddOption(autoPath, new pathplanner::PathPlannerAuto(std::string{autoPath}));
    for (int i = 0; i < 10; i++) {
      std::cout << "Constructing path:" << std::string{autoPath} << std::endl;
    }
    std::cout << "\n";
  }
  m_chooser.AddOption("Dumb Two Score Mid", new TwoNoteAuto(m_swerveDrive, m_intake, m_shooter, AutoConstants::kStartingPosition));
  m_chooser.AddOption("Mobility", new MobilityAuto(m_swerveDrive));
  frc::SmartDashboard::PutData("Auto Routines", &m_chooser);

  // Logging callbacks for pathplanner -- current pose, target pose, and active path
  pathplanner::PathPlannerLogging::setLogCurrentPoseCallback([this](frc::Pose2d pose) {
    // Do whatever you want with the poses here
    m_poseEstimator->SetCurrentAutoPose(pose);
  });

  pathplanner::PathPlannerLogging::setLogTargetPoseCallback([this](frc::Pose2d pose) {
    // Do whatever you want with the poses here
    m_poseEstimator->SetTargetAutoPose(pose);
  });

  // Logging callback for the active path, this is sent as a vector of poses
  pathplanner::PathPlannerLogging::setLogActivePathCallback([this](std::vector<frc::Pose2d> poses) {
    // Do whatever you want with the poses here
    m_poseEstimator->SetActivePath(poses);
  });
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_chooser.GetSelected();
}

SuperstructureState RobotContainer::GetSuperstructureState() {
  return m_superstructureState;
}

bool RobotContainer::IsClimbState() {
  frc::SmartDashboard::PutBoolean("Climb mode", m_superstructureState == SuperstructureState::Climb);
  return m_superstructureState == SuperstructureState::Climb;
}

void RobotContainer::SetAllCoast() {
  m_climber->SetBrakeMode(BrakeMode::Coast);
  m_intake->SetBrakeMode(BrakeMode::Coast);
  m_shooter->SetBrakeMode(BrakeMode::Coast);
  m_swerveDrive->SetBrakeMode(BrakeMode::Coast);
}

// motors will coast along

void RobotContainer::SetAllNormalBrakeMode() {
  m_climber->SetBrakeMode(BrakeMode::Default);
  m_intake->SetBrakeMode(BrakeMode::Default);
  m_shooter->SetBrakeMode(BrakeMode::Default);
  m_swerveDrive->SetBrakeMode(BrakeMode::Default);
}

// motors will stop

BrakeMode RobotContainer::GetBrakeMode() {
  return m_climber->GetBrakeMode();
}

void RobotContainer::SetShooterState(ShooterState state) {
  m_shooter->SetState(state);
}

void RobotContainer::ConfigureTestBindings() {

  // m_swerveDrive->ToggleSlowMode();

  frc2::Trigger fullIntakeUnloadButton([this](){ return m_test.GetL1Button(); });
  fullIntakeUnloadButton.OnTrue(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{ m_shooter->SetBrakeMode(BrakeMode::Brake); m_shooter->SetRollerPower(0.0); },{m_shooter}),
    frc2::ParallelCommandGroup(
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetIntakeRotation(m_intake, IntakeState::Extended)
      ),
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(1.0_s),
        SetShooterRotation(m_shooter, ShooterState::Load)
      )
    ),
    frc2::ParallelRaceGroup(
      RunLoader(m_shooter, ShooterConstants::kLoadPower, 0.0, ShooterEndCondition::EndOnFirstDetection),
      RunIntake(m_intake, IntakeConstants::kIntakePower)
    )
  ).ToPtr());
  fullIntakeUnloadButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kUnloadTime),
      RunLoader(m_shooter, ShooterConstants::kUnloadPower, ShooterConstants::kUnloadPower)
    ),
    frc2::InstantCommand([this]{
      m_shooter->SetBrakeMode(BrakeMode::Coast);
      m_intake->SetRollerPower(0.0);
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_intake, m_shooter}),
    frc2::ParallelCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::Zero),
      SetIntakeRotation(m_intake, IntakeState::Retracted)
    )
  ).ToPtr());

  // frc2::Trigger reverseIntakeButton{m_test.Button(OperatorConstants::kButtonIDTriangle)};
  // reverseIntakeButton.OnTrue(frc2::InstantCommand([this]{
  //   m_intake->SetRollerPower(-1.0);
  // },{m_intake}).ToPtr());
  // reverseIntakeButton.OnFalse(frc2::InstantCommand([this]{
  //   m_intake->SetRollerPower(0.0);
  // },{m_intake}).ToPtr());

  frc2::Trigger resetPoseButton([this](){ return m_test.GetTouchpadButton(); });
  resetPoseButton.OnTrue(frc2::InstantCommand([this]{
    // if (frc::DriverStation::GetAlliance()) {
    //   if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
    //     m_swerveDrive->SetPose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}}, true);
    //   else
    //     m_swerveDrive->SetPose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{180.0_deg}}, true);
    // }

    if (frc::DriverStation::GetAlliance()) {
      if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
        m_swerveDrive->SetPose(frc::Pose2d{0.92_m, 5.50_m, frc::Rotation2d{0.0_deg}}, false);
      else
        m_swerveDrive->SetPose(frc::Pose2d{15.579_m, 5.50_m, frc::Rotation2d{180.0_deg}}, false);
    }
  },{m_swerveDrive}).ToPtr());

frc2::Trigger directShooterLoadButton([this](){ return m_test.GetR1Button(); });
  directShooterLoadButton.OnTrue(frc2::ParallelCommandGroup(
    Drive(&m_test, m_swerveDrive, DriveState::HeadingControl),
    frc2::SequentialCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::DirectLoad),
      RunLoader(m_shooter, ShooterConstants::kDirectLoadPower, ShooterConstants::kDirectLoadPower, ShooterEndCondition::None),
      frc2::ParallelCommandGroup(
        SetShooterRotation(m_shooter, ShooterState::Zero),
        SetIntakeRotation(m_intake, IntakeState::Retracted)
      )
    )
  ).ToPtr());
  directShooterLoadButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_driveState = DriveState::HeadingControl;
      m_swerveDrive->SetDefaultCommand(Drive(&m_test, m_swerveDrive, m_driveState));
    },{m_swerveDrive}),
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter}),
    frc2::ParallelCommandGroup(
      SetShooterRotation(m_shooter, ShooterState::Zero),
      SetIntakeRotation(m_intake, IntakeState::Retracted)
    )
    // regular drive state if we don't want to directly load
  ).ToPtr());

 frc2::Trigger unloadButton([this](){ return m_test.GetCreateButton(); });
  unloadButton.OnTrue(RunLoader(m_shooter, ShooterConstants::kUnloadPower, ShooterConstants::kUnloadPower).ToPtr());
  unloadButton.OnFalse(
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter}).ToPtr()
  );
  // Old swerve align method
  // frc2::Trigger swerveAlignButton{m_test.Button(OperatorConstants::kButtonIDLeftTrigger)};
  // swerveAlignButton.ToggleOnTrue(Drive(&m_bill, m_swerveDrive, DriveState::SpeakerAlign).ToPtr());
  // swerveAlignButton.OnTrue(frc2::InstantCommand([this]{
  //   if (m_driveState == DriveState::HeadingControl) {
  //     m_driveState = DriveState::SpeakerAlign;
  //     // frc::SmartDashboard::PutString("Drive state", "Speaker align");
  //   } else {
  //     m_driveState = DriveState::HeadingControl;
  //     // frc::SmartDashboard::PutString("Drive state", "Heading control");
  //   }
  //     m_swerveDrive->SetDefaultCommand(Drive(&m_test, m_swerveDrive, m_driveState));
  //   },{m_swerveDrive}).ToPtr()
  // );
  
  // New swerve align method
  frc2::Trigger swerveAlignButtonTranslation([this](){ return m_test.GetL2Button(); });
  swerveAlignButtonTranslation.OnTrue(frc2::InstantCommand([this]{
    // if (m_driveState == DriveState::HeadingControl) {
      m_driveState = DriveState::SpeakerAlignTranslationAlgorithm;
    // } else {
    //   m_driveState = DriveState::HeadingControl;
    // }
      m_swerveDrive->SetDefaultCommand(Drive(&m_test, m_swerveDrive, m_driveState));
    },{m_swerveDrive}).ToPtr()
  );
  swerveAlignButtonTranslation.OnFalse(frc2::InstantCommand([this]{
      m_driveState = DriveState::HeadingControl;
      m_swerveDrive->SetDefaultCommand(Drive(&m_test, m_swerveDrive, m_driveState));
    },{m_swerveDrive}).ToPtr()
  );

  frc2::Trigger shootLowButton([this](){ return m_test.GetOptionsButton(); });
  shootLowButton.OnTrue(frc2::SequentialCommandGroup(
    SetShooterRotation(m_shooter, ShooterState::Mid),
    RunShooter(m_shooter, 0.27)
  ).ToPtr());
  shootLowButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kShootTime),
      RunLoader(m_shooter, 0.27, 0.27)
    ),
    SetShooterRotation(m_shooter, ShooterState::Zero),
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter})
  ).ToPtr());

  frc2::Trigger arbitraryShootButton([this](){ return m_test.GetR2Button(); });
  arbitraryShootButton.OnTrue(frc2::SequentialCommandGroup(
    SetShooterRotation(m_shooter, ShooterState::ArbitraryAngle),
    RunShooter(m_shooter, ShooterConstants::kShootPower)
  ).ToPtr());
  arbitraryShootButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kShootTime),
      RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
    ),
    SetShooterRotation(m_shooter, ShooterState::Zero),
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter})
  ).ToPtr());

  frc2::Trigger interpolateShootButton([this](){ return m_test.GetCircleButton(); });
  interpolateShootButton.OnTrue(frc2::SequentialCommandGroup(
    SetShooterRotation(m_shooter, ShooterState::InterpolateAngle),
    RunShooter(m_shooter, ShooterConstants::kShootPower)
  ).ToPtr());
  interpolateShootButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(ShooterConstants::kShootTime),
      RunLoader(m_shooter, ShooterConstants::kShootPower, ShooterConstants::kShootPower)
    ),
    SetShooterRotation(m_shooter, ShooterState::Zero),
    frc2::InstantCommand([this]{
      m_shooter->SetRollerPower(0.0);
      m_shooter->SetLoaderPower(0.0);
    },{m_shooter})
  ).ToPtr());
  
  frc2::Trigger extendBothClimbersButton([this](){ return m_test.GetTriangleButton(); });
  extendBothClimbersButton.OnTrue(frc2::SequentialCommandGroup(
    RunClimber(m_climber, ClimberConstants::kBothExtendPower, ClimberConstants::kBothExtendPower)
  )
    // .OnlyIf([this](){ return IsClimbState(); }));
    .ToPtr());
  extendBothClimbersButton.OnFalse(frc2::InstantCommand([this]{
      m_climber->SetPower(0.0, 0.0);
  },{m_climber})
    // .OnlyIf([this](){ return IsClimbState(); }));
    .ToPtr());

  frc2::Trigger retractBothClimbersButton([this](){ return m_test.GetCrossButton(); });
  retractBothClimbersButton.OnTrue(RunClimber(m_climber, ClimberConstants::kBothRetractPower, ClimberConstants::kBothRetractPower)
    // .OnlyIf([this](){ return IsClimbState(); }));
    .ToPtr());
  retractBothClimbersButton.OnFalse(frc2::InstantCommand([this]{
      m_climber->SetPower(0.0, 0.0);
  },{m_climber})
    // .OnlyIf([this](){ return IsClimbState(); }));
    .ToPtr());
//   frc2::Trigger driveRelativePosX{m_test.Button(OperatorConstants::kButtonIDSquare)};
//   driveRelativePosX.OnTrue(frc2::RunCommand([this]{
//     frc::ChassisSpeeds relativeSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(1.0_mps, 0.0_mps, 0.0_rad / 1.0_s, frc::Rotation2d{0.0_deg});
//     m_swerveDrive->DriveRobotRelative(relativeSpeeds);
//   },{m_swerveDrive}).ToPtr());
//   driveRelativePosX.OnFalse(frc2::InstantCommand([this]{
//     frc::ChassisSpeeds relativeSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(0.0_mps, 0.0_mps, 0.0_rad / 1.0_s, frc::Rotation2d{0.0_deg});
//     m_swerveDrive->DriveRobotRelative(relativeSpeeds);
//   },{m_swerveDrive}).ToPtr());

//   frc2::Trigger driveRelativePosY{m_test.Button(OperatorConstants::kButtonIDCircle)};
//   driveRelativePosY.OnTrue(frc2::RunCommand([this]{
//     frc::ChassisSpeeds relativeSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(0.0_mps, 1.0_mps, 0.0_rad / 1.0_s, frc::Rotation2d{0.0_deg});
//     m_swerveDrive->DriveRobotRelative(relativeSpeeds);
//   },{m_swerveDrive}).ToPtr());
//   driveRelativePosY.OnFalse(frc2::InstantCommand([this]{
//     frc::ChassisSpeeds relativeSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(0.0_mps, 0.0_mps, 0.0_rad / 1.0_s, frc::Rotation2d{0.0_deg});
//     m_swerveDrive->DriveRobotRelative(relativeSpeeds);
//   },{m_swerveDrive}).ToPtr());
}


// no matter how nice ethan might seem when you least expect it he will slap you with a piece of chicken and eat you in a bucket