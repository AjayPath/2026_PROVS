package frc.robot.auto;

public enum AutoMode {
  RED_RIGHT("Red Right", 90.0, false),
  RED_LEFT("Red Left", 270.0, false),
  BLUE_RIGHT("Blue Right", 270.0, true),
  BLUE_LEFT("Blue Left", 90.0, true);

  private final String label;
  private final double startingHeadingDeg;
  private final boolean blueAlliance;

  AutoMode(String label, double startingHeadingDeg, boolean blueAlliance) {
    this.label = label;
    this.startingHeadingDeg = startingHeadingDeg;
    this.blueAlliance = blueAlliance;
  }

  public String getLabel() {
    return label;
  }

  public double getStartingHeadingDeg() {
    return startingHeadingDeg;
  }

  public boolean isBlueAlliance() {
    return blueAlliance;
  }
}
