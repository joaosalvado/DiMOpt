#!/usr/bin/env bash

cp -a missions/* bin/
cd bin/
mpirun -np 5 --use-hwthread-cpus --oversubscribe ./distributed_scp simple_example 0.5pol.png
cd ..
