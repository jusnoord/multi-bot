#!/bin/bash
# deploys code to defined device through rsync, and then runs the simulation with hwSim flag
# keep in mind that this requires ssh keys to be set up for passwordless login
cd /mnt/c/Users/maxwe/diffy-swerve-from-scratch-1
URL="sparky@10.93.12.2" #todo: fix
DESTINATION="/home/sparky/diffy-swerve-from-scratch/"
echo $pwd
echo "Deploying to $URL:$DESTINATION"
rsync -avz --exclude-from='.rsync-exclude' ./ sparky@10.93.12.2:/home/sparky/diffy-swerve-from-scratch/
echo "Deployment complete!"

echo "running remote sim and piping output to terminal..."
ssh -t "sparky@10.93.12.2" "cd ./diffy-swerve-from-scratch && ./gradlew simulateJava -PhwSim"
