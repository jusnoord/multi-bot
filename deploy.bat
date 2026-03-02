@echo off
REM Deploys code to defined device through rsync, and then runs the simulation with hwSim flag
REM Requires ssh keys to be set up for passwordless login

set "URL=sparky@10.93.12.2"
set "DESTINATION=/home/sparky/diffy-swerve-from-scratch/"

echo Deploying to %URL%:%DESTINATION%
rsync -avz --exclude-from='.rsync-exclude' .\ %URL%:%DESTINATION%
echo Deployment complete!

echo Running remote sim and piping output to terminal...
ssh -t $URL "cd /home/sparky/diffy-swerve-from-scratch/ && ./gradlew simulateJava -PhwSim"