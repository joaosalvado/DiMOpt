#!/usr/bin/env bash

cd ..
cp -a missions/* bin/
cd bin/
mpirun -np 20 --use-hwthread-cpus --oversubscribe ./distributed_scp square_sided_20 0.5pol.png
cd ..
