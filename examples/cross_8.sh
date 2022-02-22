#!/usr/bin/env bash

cd ..
cp -a missions/* bin/
cd bin/
mpirun -np 8 --use-hwthread-cpus --oversubscribe ./distributed_scp cross_8 0.5pol.png
cd ..
