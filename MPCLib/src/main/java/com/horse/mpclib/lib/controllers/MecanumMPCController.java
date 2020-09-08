package com.horse.mpclib.lib.controllers;

import com.horse.mpclib.lib.control.MPCSolver;
import com.horse.mpclib.lib.control.RunnableMPC;
import com.horse.mpclib.lib.physics.MecanumDriveModel;

public class MecanumMPCController {
    private MPCSolver mpcSolver;
    private RunnableMPC runnableMPC;
    private MecanumDriveModel mecanumDriveModel;

    public MecanumMPCController() {

    }

    public MPCSolver getMpcSolver() {
        return mpcSolver;
    }

    public void setMpcSolver(MPCSolver mpcSolver) {
        this.mpcSolver = mpcSolver;
    }

    public RunnableMPC getRunnableMPC() {
        return runnableMPC;
    }

    public void setRunnableMPC(RunnableMPC runnableMPC) {
        this.runnableMPC = runnableMPC;
    }

    public MecanumDriveModel getMecanumDriveModel() {
        return mecanumDriveModel;
    }

    public void setMecanumDriveModel(MecanumDriveModel mecanumDriveModel) {
        this.mecanumDriveModel = mecanumDriveModel;
    }
}
