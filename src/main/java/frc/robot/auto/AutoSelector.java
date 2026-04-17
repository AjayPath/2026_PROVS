package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class AutoSelector {
  private static final String RED_HUB = "Red Hub";
  private static final String BLUE_HUB = "Blue Hub";

  private final SendableChooser<AutoMode> chooser = new SendableChooser<>();

  public AutoSelector() {
    chooser.setDefaultOption(AutoMode.RED_RIGHT.getLabel(), AutoMode.RED_RIGHT);
    chooser.addOption(AutoMode.RED_LEFT.getLabel(), AutoMode.RED_LEFT);
    chooser.addOption(AutoMode.BLUE_RIGHT.getLabel(), AutoMode.BLUE_RIGHT);
    chooser.addOption(AutoMode.BLUE_LEFT.getLabel(), AutoMode.BLUE_LEFT);
  }

  public void publishToDashboard() {
    SmartDashboard.putData("Auto Selector", chooser);
  }

  public AutoMode getSelectedMode() {
    AutoMode selectedMode = chooser.getSelected();
    return selectedMode != null ? selectedMode : AutoMode.RED_RIGHT;
  }

  public double getSelectedStartingHeadingDeg() {
    return getSelectedMode().getStartingHeadingDeg();
  }

  public double getSelectedStartingX() {
    return getSelectedMode().getStartingX();
  }

  public double getSelectedStartingY() {
    return getSelectedMode().getStartingY();
  }

  public boolean isBlueSelected() {
    return getSelectedMode().isBlueAlliance();
  }

  public String getSelectedHubName() {
    return isBlueSelected() ? BLUE_HUB : RED_HUB;
  }

  public double getSelectedHubX() {
    return isBlueSelected()
        ? Constants.TurnTargetConstants.kBlueHubX
        : Constants.TurnTargetConstants.kRedHubX;
  }

  public double getSelectedHubY() {
    return isBlueSelected()
        ? Constants.TurnTargetConstants.kBlueHubY
        : Constants.TurnTargetConstants.kRedHubY;
  }
}
