# Install.cmd script for Add-In Yumi hand
echo -text "Installing smart gripper Add-In"

# Define environment variable
setenv -name SMARTGRIPPER -value $BOOTPATH

# Register path for Add_In
register -type option -description SmartGripper -path $BOOTPATH

# EIO load error handle
onerror -action goto -label ERR_LOADEIO

# Load configuration files
config -filename $BOOTPATH/config/Hand_SYS.cfg -domain SYS -replace
goto -label LOAD_EIO

# Load different EIO.cfg file according to RobotWare version
#LOAD_EIO
echo -text "Start to load EIO for RW6.05 or later"
config -filename $BOOTPATH/config/Hand_EIO_2.cfg -domain EIO -replace
goto -label CON_LOADCFG

#ERR_LOADEIO
echo -text "EIO for RW6.05 or later load failed. Try with EIO for old version"
config -filename $BOOTPATH/config/Hand_EIO.cfg -domain EIO -replace
onerror -action goto -label ERR_ECHO 
echo -text "EIO for old version loaded successfully!"
goto -label CON_LOADCFG

echo -text "EIO for RW6.05 or later loaded successfully, go to CON"
goto -label CON_LOADCFG
#CON_LOADCFG
echo -text "start to load other cfg"
config -filename $BOOTPATH/config/Hand_MMC.cfg -domain MMC -replace

# Copy file to home folder
copy -from $BOOTPATH/SmartGripper.xml -to $SYSTEM/SmartGripper.xml

# Register elog messages
register -type elogmes -domain_no 11 -min 2753 -max 2760 -prepath $BOOTPATH/language/ -postpath /SmartGripper_elogtext.xml -extopt
register -type elogtitle -prepath $BOOTPATH/language/ -postpath /SmartGripper_elogtitles.xml

include -path "$RELEASE/system/instlang.cmd"

# Goto end
goto -label END_LABEL

# Error handle
onerror -action goto -label ERR_ECHO

#ERR_ECHO
echo -text "Failed to install smart gripper Add_In!"
delay -time 2000

#END_LABEL