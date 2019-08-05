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
    CONST speeddata v25:=[25,500,5000,1000];
    CONST robtarget Target_10:=[[286.15,-396.03,199.18],[0.0205163,0.803912,0.17842,0.566984],[0,0,0,4],[-134.117,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_20:=[[350.18,-260.80,-69.18],[0.00584651,-0.918747,-0.370666,-0.135928],[1,0,0,5],[-164.377,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_30:=[[336.10,-343.79,-43.91],[0.0201349,0.999667,-0.0131096,0.00938577],[1,1,0,4],[118.682,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_40:=[[335.56,-343.44,-98.24],[0.0212883,0.999464,-0.00932019,0.0230697],[1,1,0,4],[126.706,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_50:=[[541.82,-103.07,-3.14],[0.0390147,-0.999046,0.018144,-0.00752749],[1,1,0,5],[178.364,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_60:=[[156.28,-225.84,181.82],[0.0744211,0.788811,0.121406,0.597912],[0,0,0,4],[-103.699,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_70:=[[156.28,-225.84,181.82],[0.074422,0.788811,0.121406,0.597913],[0,0,0,4],[-103.699,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_80:=[[156.28,-225.84,181.82],[0.0744208,0.788811,0.121406,0.597912],[0,0,0,4],[-103.699,9E+09,9E+09,9E+09,9E+09,9E+09]];

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
        
        ! choose to run the program of GYP
        !        WHILE FALSE DO
        path10;
        !        ENDWHILE

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
                    g_GripIn\holdForce:=10,\NoWait;
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
        MoveJ Target_10, v300, z5, toollhh\WObj:=wobj0;
        WaitTime 2;
        MoveJ Target_20, v300, z5, toollhh\WObj:=wobj0;
        WaitTime 2;
        MoveJ Target_30, v300, z5, toollhh;
        WaitTime 2;
        g_GripOut;
        MoveJ Target_40, v300, z5, toollhh;
        WaitTime 2;
        g_JogIn;
        MoveJ Target_50, v300, z5, toollhh;
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
