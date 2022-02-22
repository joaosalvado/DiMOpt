#!/usr/bin/env bash

cd ..
cp -a missions/* bin/
cd bin/
mpirun -np 5 --use-hwthread-cpus --oversubscribe ./distributed_scp simple_example 0.5pol.png
mpirun -np 1 ./scp_coupled simple_example 0.5pol.png
cd ..
