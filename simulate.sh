#!/bin/bash

# define paths
PROJECT_ROOT=$(pwd)
JNI_DIR="$PROJECT_ROOT/build/jni/release"
# JAVA_BIN="/nix/store/j29aw4vdyzglbwbx7gv3bb4nn6zxwwls-openjdk-17.0.17+10/lib/openjdk/bin/java"
JAVA_BIN="/usr/bin/java"

# env vars that point to libraries
export HALSIM_EXTENSIONS="$JNI_DIR/libhalsim_ds_socket.so:$JNI_DIR/libhalsim_gui.so"
export LD_LIBRARY_PATH="$JNI_DIR"

# resolve classpath
echo "Resolving Classpath..."

# make a temporary init script for gradle to output the classpath
INIT_SCRIPT=$(mktemp)
cat << 'EOF' > "$INIT_SCRIPT"
allprojects {
    tasks.register("getCP") {
        doLast {
            println sourceSets.main.runtimeClasspath.asPath
        }
    }
}
EOF

# run gradle using the temporary file
CLASSPATH=$(./gradlew -q -I "$INIT_SCRIPT" getCP)
rm "$INIT_SCRIPT" # Clean up the temp file

# execution
if [ -z "$CLASSPATH" ]; then
    echo "Error: Could not resolve classpath."
    exit 1
fi

echo "Starting Hardware Sim with Real DS support..."
$JAVA_BIN -cp "$CLASSPATH" frc.robot.Main
