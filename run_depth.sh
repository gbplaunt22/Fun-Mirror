#!/bin/bash
set -e

# always run from repo root
cd "$(dirname "$0")"

# compile everything in src/mirror
javac -cp "lib/*:src" src/mirror/*.java

# run your test main class
java -cp "lib/*:src" -Djava.library.path=lib mirror.DepthPresenceTest
