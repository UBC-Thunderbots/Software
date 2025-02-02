#!/usr/bin/env bash


while : 
do
	bazel run //software/thunderscope:thunderscope_main --copt=-O3 --jobs=4 \
	-- --training_mode --enable_autoref --ci_mode  
done
