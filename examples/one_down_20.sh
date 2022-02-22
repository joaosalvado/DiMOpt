#!/usr/bin/env bash

cd ..
cp -a missions/* bin/
cd bin/
mpirun -np 20 --use-hwthread-cpus --oversubscribe ./distributed_scp one_down_20 0.5pol.png
cd ..
