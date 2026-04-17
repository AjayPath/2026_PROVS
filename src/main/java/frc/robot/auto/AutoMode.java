package frc.robot.auto;

public enum AutoMode {
  RED_RIGHT("Red Right", 90.0, -12.2, -7.3, false),
  RED_LEFT("Red Left", 270.0, -12.2, -0.6, false),
  BLUE_RIGHT("Blue Right", 270.0, -4.152, -0.7, true),
  BLUE_LEFT("Blue Left", 90.0, -4.25, -7.4, true);

  private final String label;
  private final double startingHeadingDeg;
  private final double startingX;
  private final double startingY;
  private final boolean blueAlliance;

  AutoMode(
      String label,
      double startingHeadingDeg,
      double startingX,
      double startingY,
      boolean blueAlliance) {
    this.label = label;
    this.startingHeadingDeg = startingHeadingDeg;
    this.startingX = startingX;
    this.startingY = startingY;
    this.blueAlliance = blueAlliance;
  }

  public String getLabel() {
    return label;
  }

  public double getStartingHeadingDeg() {
    return startingHeadingDeg;
  }

  public double getStartingX() {
    return startingX;
  }

  public double getStartingY() {
    return startingY;
  }

  public boolean isBlueAlliance() {
    return blueAlliance;
  }
}
