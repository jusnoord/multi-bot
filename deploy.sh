#!/bin/bash
# deploys code to defined device through rsync, and then runs the simulation with hwSim flag
# keep in mind that this requires ssh keys to be set up for passwordless login
URL_MASTER="sparky@10.93.12.2"
URL_SLAVE="sparky@10.93.12.5" 
DESTINATION="/home/sparky/diffy-swerve-from-scratch/"
echo "$(pwd)"
echo "Deploying to ${URL_MASTER}:${DESTINATION} and ${URL_SLAVE}:${DESTINATION}"
rsync -avz --exclude-from='.rsync-exclude' ./ "${URL_MASTER}:${DESTINATION}"
rsync -avz --exclude-from='.rsync-exclude' ./ "${URL_SLAVE}:${DESTINATION}"
echo "Deployment complete!"

# echo "running remote sim and piping output to terminal..."
# ssh -t "${URL_MASTER}" "cd ./diffy-swerve-from-scratch && ./gradlew simulateJava -PhwSim" &
# ssh -t "${URL_SLAVE}" "cd ./diffy-swerve-from-scratch && ./gradlew simulateJava -PhwSim" &
# wait
