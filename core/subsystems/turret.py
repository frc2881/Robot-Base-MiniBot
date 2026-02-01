from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
from lib.components.relative_position_control_module import RelativePositionControlModule
import core.constants as constants

class Turret(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Turret

    self._hasInitialZeroReset: bool = False

    self._turret = RelativePositionControlModule(self._constants.TURRET_CONFIG)

  def periodic(self) -> None:
    self._updateTelemetry()

  def setSpeed(self, getInput: Callable[[], units.percent]) -> Command:
    return self.runEnd(
      lambda: self._turret.setSpeed(getInput() * self._constants.INPUT_LIMIT),
      lambda: self.reset()
    ).withName("Turret:SetSpeed")
  
  def setPosition(self, position: units.inches) -> Command:
    return self.run(
      lambda: self._turret.setPosition(position)
    ).withName("Turret:SetPosition")
  
  def getPosition(self) -> units.inches:
    return self._turret.getPosition()

  def isAtTargetPosition(self) -> bool:
    return self._turret.isAtTargetPosition()

  def resetToHome(self) -> Command:
    return self._turret.resetToHome(self).withName("Turret:ResetToHome")

  def iHomed(self) -> bool:
    return self._turret.isHomed()

  def reset(self) -> None:
    self._turret.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Turret/IsAtTargetPosition", self.isAtTargetPosition())
