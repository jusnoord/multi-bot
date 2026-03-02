#!/usr/bin/env fish
# deploys code to defined device through rsync, and then runs the simulation with hwSim flag
# keep in mind that this requires ssh keys to be set up for passwordless login

if test $argv = "master"
    echo "Setting up for master robot deployment..."
    set -g IP_ADDRESS 10.93.12.2
else if test $argv = "slave"
    echo "Setting up for slave robot deployment..."
    set -g IP_ADDRESS 10.93.12.5
else
    echo "Error: Invalid argument. Please specify 'slave' or 'master'."
    exit 1
end

set URL sparky@$IP_ADDRESS #todo: fix
set DESTINATION /home/sparky/diffy-swerve-from-scratch/

echo "Deploying to $URL:$DESTINATION"
rsync -avz --exclude-from='.rsync-exclude' ./ $URL:$DESTINATION
echo "Deployment complete!"

echo "running remote sim and piping output to terminal..."
ssh -t -t $URL "cd /home/sparky/diffy-swerve-from-scratch/ && ./gradlew simulateJava -PhwSim"