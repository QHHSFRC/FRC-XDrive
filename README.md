# FRC-XDrive
This is a formula that we've developed to run an X-Drive (optionally field-centric) in c++, though it would be easily translatable to Java. Unlike the common X-Drive program that simply combines the control axes, this does not result in attempting to give a motor controller a speed out of its range (e.g. setting a -1 to 1 motor controller to a speed of 1.5). Included in this repository are a Variable library (in the Formula file which also provides a link to an easier-to-read, written-out version of the formula on the Desmos Online Graphing Calculator) and an example of how this formula can be integrated. The example begins as a field-centric x-drive controlled robot and can be switched to robot-centric when a button is pressed. The sensitivity of turning and x/y movement can be adjusted by two constants (explained in the notes of the example code).

If you have any questions/problems/comments or simply want to let us know that this helped you in any way, please contact us at qhhsfrc@gmail.com