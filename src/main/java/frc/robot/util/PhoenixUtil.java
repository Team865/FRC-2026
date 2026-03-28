// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
//
// This file was modified to log the error if the operation fails for tryUntilOk

package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static boolean tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    StatusCode lastError = command.get();
    if (lastError.isOK()) return true;

    for (int i = 0; i < maxAttempts - 1; i++) {
      lastError = command.get();
      if (lastError.isOK()) return true;
    }
    System.err.printf("Failed to apply config for %s: %s\n", command, lastError.getDescription());
    return true;
  }
}
