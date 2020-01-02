MODULE ROS_motion_right_gripper


LOCAL VAR robtarget p2 := [[609.77,-145.88,185.96],[0.174011,-0.87904,-0.130607,-0.424207],[1,1,0,5],[-157.635,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget p1 := [[289.29,-251.60,66.17],[0.1754,-0.935396,-0.193132,-0.238684],[0,1,0,4],[-176.388,9E+09,9E+09,9E+09,9E+09,9E+09]];    ! Software License Agreement (BSD License)
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
    LOCAL CONST zonedata DEFAULT_CORNER_DIST:=z10;
    LOCAL CONST num GRIPPER_POS_TOL:=2;

    ! ----------------------------------------------
    ! ----------------- VARIABLES ------------------
    ! ----------------------------------------------
    ! Trajectory Variables
    LOCAL VAR ROS_joint_trajectory_pt jointTrajectory{MAX_TRAJ_LENGTH};
    LOCAL VAR ROS_gripper_trajectory_pt gripperTrajectory{MAX_TRAJ_LENGTH};
    LOCAL VAR num newGripperPos;
    ! store new gripper position locally
    LOCAL VAR num previousGripperPos;
    ! store old gripper position locally
    LOCAL VAR num trajectory_size:=0;

    ! Flag Variables
    LOCAL VAR bool flag_handCalibrated:=FALSE;
    LOCAL VAR bool flag_programStarted:=FALSE;

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
    CONST speeddata v_user:=[1500,500,5000,1500];
    CONST robtarget Target_10:=[[101.01,-327.76,157.47],[0.10506,0.864165,0.109164,0.479858],[0,0,0,4],[-102.342,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_20:=[[139.77,-362.64,127.32],[0.253561,-0.0570428,0.963772,0.0599738],[0,3,0,5],[177.931,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_30:=[[251.94,-392.88,18.27],[0.017167,-0.00570527,-0.999817,-0.00620149],[1,3,0,4],[-174.78,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_40:=[[251.95,-392.86,-122.66],[0.0171773,-0.00570583,-0.999817,-0.00622844],[1,3,0,4],[-173.943,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_50:=[[251.96,-392.86,-7.77],[0.0171805,-0.00571558,-0.999817,-0.00624336],[1,3,0,4],[-173.94,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_60:=[[247.81,-382.01,24.65],[0.0123878,0.99914,-0.0384922,-0.00919415],[1,0,0,5],[-115.46,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_70:=[[367.01,-326.75,9.19],[0.000596202,0.00834558,-0.927731,-0.373155],[1,3,0,4],[-85.5333,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_80:=[[384.14,-15.82,-125.06],[0.000592106,0.0083172,-0.92773,-0.373158],[1,2,0,4],[-74.5483,9E+09,9E+09,9E+09,9E+09,9E+09]];
     CONST robtarget Target_90:=[[646.04,-333.01,226.70],[0.373386,0.608055,0.403421,0.572803],[1,-1,1,4],[-127.059,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_100:=[[663.12,-237.82,200.24],[0.26791,0.656608,0.281957,0.646212],[1,-2,2,4],[-108.549,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_110:=[[642.12,-4.60,164.53],[0.165632,0.667651,0.250028,0.681391],[2,-1,1,4],[-79.8268,9E+09,9E+09,9E+09,9E+09,9E+09]];

    ! The const difined/modified by Honghao Lv for calibration
    CONST jointtarget Phome:=[[0,-130,30,0,40,0],[-135,0,0,0,0,0]];
CONST robtarget ptest:=[[165.55,-319.44,200.01],[0,1,0,0],[0,-2,2,4],[-101.794,9E+09,9E+09,9E+09,9E+09,9E+09]];

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
        VAR zonedata stop_mode;
        VAR bool skip_move;

        ! for GYP lhh
        CONNECT intno1 WITH Path_V0;
        ISignalDI custom_DI_1,1,intno1;

        ! Syncronize Tasks
        IF (flag_programStarted=FALSE) THEN
!            WaitSyncTask ready,task_list\TimeOut:=20;
            flag_programStarted:=TRUE;
        ELSE
            TPWrite task_name+": Program restarted.";
        ENDIF

        IF (flag_handCalibrated=FALSE) THEN
            calibrate_hand;
            ! Calibrate the hand
!            WaitSyncTask handCalibrated,task_list\TimeOut:=20;
            ! Hand has been calibrated
        ENDIF

        ! Make Sure YuMi Wont Start Running Previous Stored Trajectory
        ROS_new_trajectory_right:=FALSE;
        trajectory_size:=0;
        ClearPath;

        ! Move the right arm to the Calibration Home position, modified by honghao
        MoveAbsJ Phome,v100,z5,tool0\WObj:=wobj0;
        WaitTime 2;

        ! choose to run the program of GYP
!        WHILE TRUE DO
!        path10;
!        ENDWHILE

        ! Get Trajectory and Move Arm
        WHILE true DO
            ! Check For New Trajectory
            IF (ROS_new_trajectory_right)
            init_trajectory;

            ! Execute All Points in Trajectory
            IF (trajectory_size>0) THEN
                TPWrite task_name+": Running trajectory...";

                ! Move Gripper
                newGripperPos:=gripperTrajectory{trajectory_size}.gripper_pos;
                ! store new gripper position locally
                IF (NOT (newGripperPos=previousGripperPos)) THEN
                    IF ((newGripperPos<GRIPPER_CLOSE_TOL) AND (previousGripperPos>(newGripperPos+GRIPPER_CLOSE_TOL))) THEN
                        ! if gripping an object
                        Hand_GripInward\holdForce:=10,\NoWait;
                        ! grip object without waiting for movement to complete
                    ELSEIF (newGripperPos>GRIPPER_CLOSE_TOL) THEN
                        ! if not gripping an object
                        Hand_MoveTo newGripperPos,\NoWait;
                        ! go to the gripper position without waiting for movement to complete
                    ENDIF
                    previousGripperPos:=newGripperPos;
                ENDIF

                ! Move Arm
                FOR current_index FROM 1 TO trajectory_size DO
                    target.robax:=jointTrajectory{current_index}.joint_pos.robax;
                    target.extax.eax_a:=jointTrajectory{current_index}.joint_pos.extax.eax_a;

                    skip_move:=(current_index=1) AND is_near(target.robax,0.1);

                    ! If At Final Point, Make Fine Movement, Otherwise Use Zone Movements
                    IF (current_index=trajectory_size) THEN
                        stop_mode:=fine;
                        ! stop at path end
                    ELSE
                        stop_mode:=DEFAULT_CORNER_DIST;
                        ! assume we are smoothing between points
                    ENDIF

                    ! Execute move command
                    IF (NOT skip_move) THEN
                        ! MoveAbsJ target, NORM_SPEED, \T:=jointTrajectory{current_index}.duration, stop_mode, tool0; ! move arm
                        ! modified by lhh
                        MoveAbsJ target,NORM_SPEED,stop_mode,tool0;
                    ENDIF
                ENDFOR
                trajectory_size:=0;
                ! trajectory done
                StopMove;
                ClearPath;
                ! clear movement buffer for arm

                TPWrite task_name+": Finished Trajectory.";
            ENDIF

            WaitTime 0.05;
            ! throttle loop while waiting for new command
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

    LOCAL PROC init_trajectory()
        ! MODIFIER: Frederick Wachter - wachterfreddy@gmail.com
        ! FIRST MODIFIED: 2016-06-14
        ! PURPOSE: Get joint and gripper trajectory
        ! NOTES: A gripper is attached to this arm (right arm)

        clear_path;
        ! cancel any active motions

        ! Get Trajectory For Arm
        WaitTestAndSet ROS_trajectory_lock_right;
        ! acquire data-lock
        trajectory_size:=ROS_trajectory_size_right;
        ! get the trajectory size
        jointTrajectory:=ROS_trajectory_right;
        ! copy joint trajectory to local variable
        gripperTrajectory:=ROS_trajectory_gripper_r;
        ! copy joint trajectory to local variable

        ROS_new_trajectory_right:=FALSE;
        ! set flag to indicate that the new trajectory has already been retrived
        ROS_trajectory_lock_right:=FALSE;
        ! release data-lock

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

    LOCAL FUNC bool is_near(robjoint target,num tol)
        VAR jointtarget curr_jnt;

        curr_jnt:=CJointT();

        RETURN (ABS(curr_jnt.robax.rax_1-target.rax_1)<tol)
       AND (ABS(curr_jnt.robax.rax_2-target.rax_2)<tol)
       AND (ABS(curr_jnt.robax.rax_3-target.rax_3)<tol)
       AND (ABS(curr_jnt.robax.rax_4-target.rax_4)<tol)
       AND (ABS(curr_jnt.robax.rax_5-target.rax_5)<tol)
       AND (ABS(curr_jnt.robax.rax_6-target.rax_6)<tol);
    ENDFUNC

    LOCAL PROC abort_trajectory()
        trajectory_size:=0;
        ! "clear" local trajectory
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
!        MoveJ Target_10,v_user,z5,toollhh\WObj:=wobj0;
!        WaitTime 1;
!        MoveJ Target_20,v_user,z5,toollhh\WObj:=wobj0;
!        WaitTime 1;
!        MoveJ Target_30,v_user,z5,toollhh;
!        WaitTime 2;
!        g_GripOut;
!        MoveL Target_40,v_user,z5,toollhh;
!        WaitTime 2;
!        g_GripIn\holdForce:=20;
!        WaitTime 1;
!        MoveL Target_50,v_user,z5,toollhh;
        !        MoveJ Target_60, v_user, z5, toollhh;
!        MoveJ Target_70,v_user,z5,toollhh;
!        WaitTime 1;
!        MoveL Target_80,v_user,z5,toollhh;
!        WaitTime 2;
!        g_GripOut;
!        WaitTime 1;
!        MoveL Target_70,v_user,z5,toollhh;
!        g_GripIn;

! 2019.11.14 for GYP
        !TPWrite "Max. TCP speed in mm/s for my robot="\Num:=MaxRobSpeed();
        MoveJ Target_90,v300,z5,toollhh;
        !MoveL Target_100,v300,z5,toollhh;
        MoveL Target_110,v_user,z5,toollhh;

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
