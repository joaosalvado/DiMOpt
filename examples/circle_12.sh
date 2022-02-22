#!/usr/bin/env bash

cd ..
cp -a missions/* bin/
cd bin/
mpirun -np 12 --use-hwthread-cpus --oversubscribe ./distributed_scp circle_12 0.5pol.png
cd ..
