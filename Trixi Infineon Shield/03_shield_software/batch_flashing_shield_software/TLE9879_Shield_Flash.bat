@ECHO OFF
::***********************Download Relais Test********************************
Jlink.exe -speed 4000 -CommanderScript all_applications.jlinkscript
timeout 1

SET /P q=Continue, press key