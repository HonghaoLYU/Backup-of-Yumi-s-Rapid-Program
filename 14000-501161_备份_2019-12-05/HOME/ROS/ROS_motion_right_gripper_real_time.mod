MODULE ROS_motion_r_gripper_rt

    ! Software License Agreement (BSD License)
    !
    ! Copyright (c) 2012, Edward Venator, Case Western Reserve University
    ! Copyright (c) 2012, Jeremy Zoss, Southwest Research Institute
    ! All rights reserved.
    !
    ! Redistribution and use in source and binary forms, with or without modification,
    ! are permitted provided that the following conditions are met:
    !
    !   Redistributions of source code must retain the above copyright notice, this
    !       list of conditions and the following disclaimer.
    !   Redistributions in binary form must reproduce the above copyright notice, this
    !       list of conditions and the following disclaimer in the documentation
    !       and/or other materials provided with the distribution.
    !   Neither the name of the Case Western Reserve University nor the names of its contributors
    !       may be used to endorse or promote products derived from this software without
    !       specific prior written permission.
    !
    ! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
    ! EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
    ! OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
    ! SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    ! INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
    ! TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
    ! BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    ! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
    ! WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    ! ----------------------------------------------
    ! ----------------- CONSTANTS ------------------
    ! ----------------------------------------------
    LOCAL CONST zonedata DEFAULT_CORNER_DIST:=z1;
    LOCAL CONST num GRIPPER_POS_TOL:=2;

    ! ----------------------------------------------
    ! ----------------- VARIABLES ------------------
    ! ----------------------------------------------
    ! Trajectory Variables
    LOCAL VAR ROS_joint_trajectory_pt jointTrajectory;
    LOCAL VAR num newGripperPos;
    ! store new gripper position locally
    LOCAL VAR num previousGripperPos;
    ! store old gripper position locally

    ! Flag Variables
    LOCAL VAR bool flag_handCalibrated:=FALSE;
    LOCAL VAR bool flag_programStarted:=FALSE;
    LOCAL VAR bool flag_firstPoint:=FALSE;

    ! Task Name
    LOCAL VAR string task_name:="M_Right";

    ! Syncronize Motion Variables
    PERS tasks task_list{2}:=[["T_ROB_R"],["T_ROB_L"]];
    VAR syncident ready;
    VAR syncident handCalibrated;
    TASK PERS tooldata toollhh:=[TRUE,[[7.97617,3.97063,125.858],[1,0,0,0]],[0.05,[0,0,10],[1,0,0,0],0,0,0]];
    TASK PERS wobjdata wobjlhh:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

    ! difined for Gaoyang Pang, modified by honghao lv 190805
    VAR intnum intno1;
    VAR intnum intno2;
    VAR speeddata v:=v50;
    CONST speeddata v_user:=v300;
    CONST robtarget Target_10:=[[101.01,-327.76,157.47],[0.10506,0.864165,0.109164,0.479858],[0,0,0,4],[-102.342,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_20:=[[139.77,-362.64,127.32],[0.253561,-0.0570428,0.963772,0.0599738],[0,3,0,5],[177.931,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_30:=[[251.94,-392.88,18.27],[0.017167,-0.00570527,-0.999817,-0.00620149],[1,3,0,4],[-174.78,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_40:=[[251.95,-392.86,-122.66],[0.0171773,-0.00570583,-0.999817,-0.00622844],[1,3,0,4],[-173.943,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_50:=[[251.96,-392.86,-7.77],[0.0171805,-0.00571558,-0.999817,-0.00624336],[1,3,0,4],[-173.94,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_60:=[[247.81,-382.01,24.65],[0.0123878,0.99914,-0.0384922,-0.00919415],[1,0,0,5],[-115.46,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_70:=[[367.01,-326.75,9.19],[0.000596202,0.00834558,-0.927731,-0.373155],[1,3,0,4],[-85.5333,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_80:=[[384.14,-15.82,-125.06],[0.000592106,0.0083172,-0.92773,-0.373158],[1,2,0,4],[-74.5483,9E+09,9E+09,9E+09,9E+09,9E+09]];

    ! The const difined/modified by Honghao Lv for calibration
    CONST jointtarget Phome:=[[0,-130,30,0,40,0],[-135,0,0,0,0,0]];

    PROC main()
        ! MODIFIER: Frederick Wachter - wachterfreddy@gmail.com
        ! FIRST MODIFIED: 2016-06-14
        ! PURPOSE: Move the robot arm and gripper to specified trajectory
        ! NOTES: A gripper is attached to this arm (right arm)
        ! FUTURE WORK: Use the velociy values, need to find a way to send grip command to run Hand_GripInward function properly

        ! Initialize Variables
        VAR num current_index;
        VAR jointtarget target;
        VAR num gripperTarget;

        ! for GYP lhh
        CONNECT intno1 WITH Path_V0;
        ISignalDI custom_DI_1,1,intno1;

        ! Syncronize Tasks
        IF (flag_programStarted=FALSE) THEN
            WaitSyncTask ready,task_list\TimeOut:=20;
            flag_programStarted:=TRUE;
        ELSE
            TPWrite task_name+": Program restarted.";
            flag_firstPoint:=TRUE;
        ENDIF

        IF (flag_handCalibrated=FALSE) THEN
            calibrate_hand;
            ! Calibrate the hand
            WaitSyncTask handCalibrated,task_list\TimeOut:=20;
            ! Hand has been calibrated
        ENDIF

        ! Make Sure YuMi Wont Start Running Previous Stored Trajectory
        ROS_new_trajectory_right:=FALSE;
        ClearPath;

        ! Move the right arm to the Calibration Home position, modified by honghao
        MoveAbsJ Phome,v100,z5,tool0\WObj:=wobj0;
        WaitTime 2;

        ! choose to run the program of GYP
        !WHILE TRUE DO
            !path10;
        !ENDWHILE

        ! Wait For First Trajectory To Be Sent
        WHILE (NOT (flag_firstPoint)) DO
            IF (ROS_new_trajectory_right) THEN
                flag_firstPoint:=TRUE;
            ENDIF
        ENDWHILE

        ! Get Trajectories and Move Arm
        WHILE true DO
            ! Move Gripper
            newGripperPos:=ROS_trajectory_gripper_r{1}.gripper_pos;
            IF (NOT (newGripperPos=previousGripperPos)) THEN
                IF ((newGripperPos+GRIPPER_CLOSE_TOL)<previousGripperPos) THEN
                    ! if gripping an object, Honghao lv modified in 190423
                    g_GripIn\holdForce:=20\NoWait;
                    ! grip object without waiting for movement to complete, Honghao lv modified in 190423
                ELSEIF (newGripperPos>previousGripperPos-GRIPPER_CLOSE_TOL) THEN
                    ! if not gripping an object
                    g_MoveTo newGripperPos,\NoWait;
                    ! go to the gripper position without waiting for movement to complete
                ENDIF
                previousGripperPos:=newGripperPos;
            ENDIF

            ! Move Arm
            jointTrajectory:=ROS_trajectory_right{1};
            target.robax:=jointTrajectory.joint_pos.robax;
            target.extax.eax_a:=jointTrajectory.joint_pos.extax.eax_a;

            MoveAbsJ target,v100,\T:=0.01,DEFAULT_CORNER_DIST,tool0;
            ! move arm

        ENDWHILE
    ERROR (ERR_WAITSYNCTASK)
        IF (ERRNO=ERR_WAITSYNCTASK) THEN
            ErrWrite\W,"WaitSync timeout","Waited too long for hand calibration";
            TPWrite "Wait sync error";
        ELSE
            ErrWrite\W,"Motion Error","Error executing motion.  Aborting trajectory.";
            TPWrite("Error Number: "+ValToStr(ERRNO)+" | Error executing motion");
            abort_trajectory;
        ENDIF

    ENDPROC

    LOCAL PROC calibrate_hand()
        ! PROGRAMMER: Frederick Wachter - wachterfreddy@gmail.com
        ! DATE CREATED: 2016-06-14
        ! PURPOSE: Calibrate the gripper
        ! NOTES: A gripper is attached to this arm (right arm)
        ! FUTURE WORK: Fix issues with miscalibration, need to ensure hand has proper IP somehow

        ! Calibrate Hand
        Hand_JogInward;
        ! Right gripper has issues closing completely for caliration, this was added to ensure gripper closes all the way
        Hand_Initialize\maxSpd:=20,\holdForce:=10,\Calibrate;
        flag_handCalibrated:=TRUE;

        ! Notify User Hand is Calibrated
        Hand_MoveTo(10);
        Hand_MoveTo(0);
        Hand_WaitMovingCompleted;
        ! ensure the hand is at the correct location
        TPWrite "Right hand Calibrated.";

        previousGripperPos:=Hand_GetActualPos();

    ENDPROC

    LOCAL PROC abort_trajectory()
        clear_path;
        ExitCycle;
        ! restart program
    ENDPROC

    LOCAL PROC clear_path()
        IF (NOT (IsStopMoveAct(\FromMoveTask) OR IsStopMoveAct(\FromNonMoveTask))) THEN
            StopMove;
            ! stop any active motions
        ENDIF
        ClearPath;
        ! clear queued motion commands
        StartMove;
        ! re-enable motions
    ENDPROC

    ! for GYP lhh
    PROC path10()
        MoveJ Target_10,v_user,z5,toollhh\WObj:=wobj0;
        WaitTime 1;
        MoveJ Target_20,v_user,z5,toollhh\WObj:=wobj0;
        WaitTime 1;
        MoveJ Target_30,v_user,z5,toollhh;
        WaitTime 2;
        g_GripOut;
        MoveL Target_40,v_user,z5,toollhh;
        WaitTime 2;
        g_GripIn\holdForce:=20;
        WaitTime 1;
        MoveL Target_50,v_user,z5,toollhh;
        !        MoveJ Target_60, v_user, z5, toollhh;
        MoveJ Target_70,v_user,z5,toollhh;
        WaitTime 1;
        MoveL Target_80,v_user,z5,toollhh;
        WaitTime 2;
        g_GripOut;
        WaitTime 1;
        MoveL Target_70,v_user,z5,toollhh;
        g_GripIn;

    ENDPROC

    !    ! for GYP lhh
    !    TRAP Path_V0
    !        CONNECT intno2 WITH Path_V1;
    !        ISignalDI custom_DI_2,1,intno2;
    !        StopMove;
    !        StorePath;
    !        MoveL Target_20,v25,z5,toollhh\WObj:=wobj0;
    !        RestoPath;
    !    ENDTRAP

    ! for GYP lhh
    TRAP Path_V0
        StopMove;
        !        StorePath;
        !        MoveL Target_20,v10,z5,toollhh\WObj:=wobj0;
        !        RestoPath;
    ENDTRAP

ENDMODULE
