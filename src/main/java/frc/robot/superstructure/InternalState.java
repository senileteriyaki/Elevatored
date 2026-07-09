package frc.robot.superstructure;

import frc.robot.util.IState;

public enum InternalState implements IState{
    IDLE,
    DISABLED,
    BOOT,
    PRECLIMB,
    CLIMB,
    REJECT,
    PRESCORE, // track to reef, got to desired arm elbow angle
    SCORESTAGE1, // raise elevator, go to desired arm shoulder angle as well
    SCORESTAGE2, // pull arm shoulder back
    POSTSCORE // back to stow
}
