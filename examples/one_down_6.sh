#!/usr/bin/env bash

cd ..
cp -a missions/* bin/
cd bin/
mpirun -np 6 --use-hwthread-cpus --oversubscribe ./distributed_scp one_down_6 0.5pol.png
cd ..
