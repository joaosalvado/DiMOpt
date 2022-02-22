#!/usr/bin/env bash

cd ..
cp -a missions/* bin/
cd bin/
mpirun -np 3 --use-hwthread-cpus --oversubscribe ./distributed_scp take_over_3 0.5pol.png
cd ..
