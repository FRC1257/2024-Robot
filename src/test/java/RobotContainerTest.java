// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
 
import frc.robot.RobotContainer;

import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.Test;

public class RobotContainerTest {

  @Test
  public void createRobotContainer() {
    try {
      new RobotContainer();
    } catch (Exception e) {
      e.printStackTrace();
      fail("Failed to instantiate robot container - see the above stack trace for more information");
    }
  }
}