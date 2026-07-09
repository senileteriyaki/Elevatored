package frc.robot.superstructure;

import frc.robot.util.IState;

public enum Intention implements IState{
    IDLE,
    SCORE,
    PRECLIMB,
    CLIMB,
    REJECT
}
